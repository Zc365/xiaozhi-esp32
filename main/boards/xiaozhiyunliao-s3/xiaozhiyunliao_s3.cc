// #include "wifi_board.h"
#include "codecs/es8388_audio_codec.h"
#include "xiaoziyunliao_display.h"
#include "xiaozhiyunliao_s3.h"
// #include "button.h"
#include "settings.h"
#include "config.h"
// #include "font_awesome.h"
#include <wifi_station.h>
#include <ssid_manager.h>
#include <esp_log.h>
// #include "i2c_device.h"
#include <driver/i2c_master.h>
#include <string.h> 
#include <wifi_configuration_ap.h>
#include <assets/lang_config.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/spi_common.h>

#define TAG "YunliaoS3"

LV_FONT_DECLARE(font_awesome_20_4);
LV_FONT_DECLARE(font_puhui_20_4);
#define FONT font_puhui_20_4

esp_lcd_panel_handle_t panel = nullptr;

XiaoZhiYunliaoS3::XiaoZhiYunliaoS3() 
    : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, BOOT_4G_PIN),
      boot_button_(BOOT_BUTTON_PIN, false, KEY_EXPIRE_MS),
      power_manager_(new PowerManager()),
      uart_num_(UART_NUM_1),
      uart_queue_(nullptr),
      isInstallUart_(false),
      chbuf_(nullptr){
    power_manager_->Start5V();
    power_manager_->Initialize();
    InitializeI2c();
    Settings settings1("board", true);
    if(settings1.GetInt("sleep_flag", 0) > 0){
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(boot_button_.getButtonLevel() == 1) {
            Sleep(); //进入休眠模式
        }else{
            settings1.SetInt("sleep_flag", 0);
        }
    }

    InitializeButtons();
    power_manager_->OnChargingStatusDisChanged([this](bool is_discharging) {
        if(power_save_timer_){
            if (is_discharging) {
                // ESP_LOGI(TAG, "SetShutdownEnabled");
                power_save_timer_->SetShutdownEnabled(true);
            } else {
                // ESP_LOGI(TAG, "SetShutdownDisabled");
                power_save_timer_->SetShutdownEnabled(false);
            }
        }
    });

    InitializeSpi();
    InitializeLCDDisplay();

    Settings settings("aec", false);
    auto& app = Application::GetInstance();
    app.SetAecMode(settings.GetInt("mode",kAecOnDeviceSide) == kAecOnDeviceSide ? kAecOnDeviceSide : kAecOff);
    display_->ShowAEC(app.GetAecMode() != kAecOff);

    if(GetAudioCodec()->output_volume() == 0){
        GetAudioCodec()->SetOutputVolume(70);
    }
    if(GetBacklight()->RestoreBrightness() < 5){
        GetBacklight()->SetBrightness(60);
    }
    InitializePowerSaveTimer();
    InitializeModul();
    ESP_LOGI(TAG, "Inited");
}

void XiaoZhiYunliaoS3::InitializeModul() {
    chbuf_ = (char *)heap_caps_malloc(chbufSize_, MALLOC_CAP_SPIRAM);
    memset(chbuf_, 0, chbufSize_);
    // 检查4G模块是否存在，wifi模式下才需要检查并关闭4G
    if(GetNetworkType() == NetworkType::WIFI){
        // 0: 未检查过4G模块是否存在
        // 1: 4G模块不存在
        // 2: 4G模块存在
        Settings settings("board", false);
        int network_type = settings.GetInt("ok_4g", 0); 
        if(network_type == 2){
            power_manager_->Shutdown4G();
        }else if(network_type == 0){
            xTaskCreate([](void* arg) {
                auto* board = static_cast<XiaoZhiYunliaoS3*>(arg);
                XiaoZhiYunliaoS3::modultype modul_type = board->check4GModul();
                Settings settings("board", true);
                if (modul_type == XiaoZhiYunliaoS3::modultype::MODUL_4G) {
                    settings.SetInt("ok_4g", 2);
                    board->power_manager_->Shutdown4G();
                }else{
                    settings.SetInt("ok_4g", 1);
                    board->InitializeBTEmitter();
                }
                vTaskDelete(NULL);
            }, "check4g_modul", 2048, this, 5, NULL);
        }else if(network_type == 1){
            power_manager_->Start4G();
            InitializeBTEmitter();
        }
    }
}
void XiaoZhiYunliaoS3::InitializeBTEmitter() {
#if CONFIG_USE_BLUETOOTH
    if (MON_BTLINK_PIN == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "MON_BTLINK_PIN not configured, skipping GPIO interrupt setup");
        return;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << MON_BTLINK_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    if (gpio_config(&io_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d", MON_BTLINK_PIN);
        return;
    }

    m_gpio_evt_queue = xQueueCreate(3, sizeof(uint32_t));
    if (m_gpio_evt_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return;
    }

    BaseType_t ret = xTaskCreate(gpioTask, "gpio_task", 2048, this, 10, &m_gpio_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO task");
        vQueueDelete(m_gpio_evt_queue);
        m_gpio_evt_queue = nullptr;
        return;
    }

    if (gpio_isr_handler_add(MON_BTLINK_PIN, gpioIsrHandler, this) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler");
        vQueueDelete(m_gpio_evt_queue);
        m_gpio_evt_queue = nullptr;
        vTaskDelete(m_gpio_task_handle);
        m_gpio_task_handle = nullptr;
        return;
    }

    int initial_level = gpio_get_level(MON_BTLINK_PIN);
    if (initial_level == 1) {
        switchBtMode(true);
    }
    ESP_LOGI(TAG, "MON_BTLINK_PIN %d initialized, initial level: %d", MON_BTLINK_PIN, initial_level);
#endif
}

void XiaoZhiYunliaoS3::gpioIsrHandler(void *arg) {
    XiaoZhiYunliaoS3 *instance = static_cast<XiaoZhiYunliaoS3 *>(arg);
    uint32_t gpio_num = MON_BTLINK_PIN;
    xQueueSendFromISR(instance->m_gpio_evt_queue, &gpio_num, NULL);
}

void XiaoZhiYunliaoS3::gpioTask(void *arg) {
    XiaoZhiYunliaoS3 *instance = static_cast<XiaoZhiYunliaoS3 *>(arg);
    uint32_t io_num;
    int last_level = -1;

    for (;;) {
        if (xQueueReceive(instance->m_gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            int level = gpio_get_level((gpio_num_t)io_num);
            if (level != last_level) {
                last_level = level;
                
                if (level == 1) {
                    ESP_LOGI(TAG, "GPIO %d high - BT connected", io_num);
                    instance->switchBtMode(true);
                } else {
                    ESP_LOGI(TAG, "GPIO %d low - BT disconnected", io_num);
                    instance->switchBtMode(false);
                }
            }
        }
    }
    vTaskDelete(NULL);
}
void XiaoZhiYunliaoS3::deinitBTEmitter() {
    if (MON_BTLINK_PIN == GPIO_NUM_NC) {
        return;
    }

    if (MON_BTLINK_PIN != GPIO_NUM_NC) {
        gpio_isr_handler_remove(MON_BTLINK_PIN);
    }

    if (m_gpio_task_handle != nullptr) {
        vTaskDelete(m_gpio_task_handle);
        m_gpio_task_handle = nullptr;
    }

    if (m_gpio_evt_queue != nullptr) {
        vQueueDelete(m_gpio_evt_queue);
        m_gpio_evt_queue = nullptr;
    }

    gpio_reset_pin(MON_BTLINK_PIN);
}

void XiaoZhiYunliaoS3::InitializePowerSaveTimer() {
    power_save_timer_ = new PowerSaveTimer(-1, 15, 600);//修改PowerSaveTimer为sleep=idle模式, shutdown=关机模式
    power_save_timer_->OnEnterSleepMode([this]() {
        // ESP_LOGI(TAG, "Enabling idle mode");
#if CONFIG_USE_MUSIC
        auto music = GetMusic();
        if (music->IsDownloading()) {
            return; // 音乐下载中，不进入待机模式
        }
#endif
        GetDisplay()->ShowStandbyScreen(true);
        GetBacklight()->SetBrightness(GetBacklight()->brightness() / 2);
    });
    power_save_timer_->OnExitSleepMode([this]() {
        // ESP_LOGI(TAG, "Exit idle mode");
        GetDisplay()->ShowStandbyScreen(false);
        GetBacklight()->RestoreBrightness();
    });
    // power_save_timer_->OnShutdownRequest([this]() {
    //     ESP_LOGI(TAG, "Shutting down");
    //     Sleep();
    // });
    power_save_timer_->SetEnabled(true);
}

void XiaoZhiYunliaoS3::PowerSaveTimerSetEnabled(bool enabled) {
    if(power_save_timer_){
        power_save_timer_->SetEnabled(enabled);
    }
}

Backlight* XiaoZhiYunliaoS3::GetBacklight() {
    static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
    return &backlight;
}



Display* XiaoZhiYunliaoS3::GetDisplay() {
    return display_;
}

void XiaoZhiYunliaoS3::InitializeSpi() {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = DISPLAY_SPI_PIN_MOSI;
    buscfg.miso_io_num = DISPLAY_SPI_PIN_MISO;
    buscfg.sclk_io_num = DISPLAY_SPI_PIN_SCLK;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}


void XiaoZhiYunliaoS3::InitializeLCDDisplay() {
    esp_lcd_panel_io_handle_t panel_io = nullptr;
    // 液晶屏控制IO初始化
    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = DISPLAY_SPI_PIN_LCD_CS;
    io_config.dc_gpio_num = DISPLAY_SPI_PIN_LCD_DC;
    io_config.spi_mode = 3;
    io_config.pclk_hz = DISPLAY_SPI_CLOCK_HZ;
    io_config.trans_queue_depth = 10;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_LCD_HOST, &io_config, &panel_io));

    // 初始化液晶屏驱动芯片
    ESP_LOGD(TAG, "Install LCD driver");
    Settings settings("display", false);
    bool currentIpsMode = settings.GetBool("ips_mode", DISPLAY_INVERT_COLOR);
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = DISPLAY_SPI_PIN_LCD_RST;
    panel_config.bits_per_pixel = 16;
    panel_config.rgb_ele_order = DISPLAY_RGB_ORDER_COLOR;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, currentIpsMode);
    esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
    esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    display_ = new XiaoziyunliaoDisplay(
        panel_io,
        panel, 
        DISPLAY_BACKLIGHT_PIN, 
        DISPLAY_BACKLIGHT_OUTPUT_INVERT,
        DISPLAY_WIDTH, 
        DISPLAY_HEIGHT,
        DISPLAY_OFFSET_X, 
        DISPLAY_OFFSET_Y,
        DISPLAY_MIRROR_X, 
        DISPLAY_MIRROR_Y, 
        DISPLAY_SWAP_XY);
}

void XiaoZhiYunliaoS3::InitializeI2c() {
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
        .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
}

void XiaoZhiYunliaoS3::InitializeButtons() {
    boot_button_.OnClick([this]() {
        // ESP_LOGI(TAG, "Button OnClick");
        auto& app = Application::GetInstance();
        app.ToggleChatState();
        //单击按键唤醒不再响应
        // std::string wake_word=Lang::Strings::WAKE_WORD;
        // app.WakeWordInvoke(wake_word);
    });
    boot_button_.OnPressDown([this]() {
        // ESP_LOGI(TAG, "Button OnPressDown");
        power_save_timer_->WakeUp();
    });
    boot_button_.OnPressUp([this]() {
        // ESP_LOGI(TAG, "Button OnPressUp");
    });
    boot_button_.OnLongPress([this]() {
        // ESP_LOGI(TAG, "Button LongPres");
        display_->SetStatus(Lang::Strings::SHUTTING_DOWN);
        display_->HideChatPage();
        display_->HideSmartConfigPage();
        display_->DelConfigPage();
        vTaskDelay(pdMS_TO_TICKS(2000));
        Sleep();
    });    
    boot_button_.OnDoubleClick([this]() {
        // ESP_LOGI(TAG, "Button OnDoubleClick");
        if (display_ && !wifi_config_mode_) {
            display_->SwitchPage();
        }
    });  
    boot_button_.OnThreeClick([this]() {
        ESP_LOGI(TAG, "Button OnThreeClick");
#if CONFIG_USE_DEVICE_AEC
        if (display_->GetPageIndex() == PageIndex::PAGE_CONFIG) {
            auto& app = Application::GetInstance();
            bool enableAec = app.GetAecMode() == kAecOff;
            switchAecMode(enableAec);
            display_->SwitchPage();
            return;
        }
#endif
        SwitchNetwork();
    });  
    boot_button_.OnFourClick([this]() {
        ESP_LOGI(TAG, "Button OnFourClick");
        if (display_->GetPageIndex() == PageIndex::PAGE_CONFIG) {
            ClearWifiConfiguration();
        }
        if (GetWifiConfigMode()) {
            SetFactoryWifiConfiguration();
        }
    });
    boot_button_.OnFiveClick([this]() {
        ESP_LOGI(TAG, "Button OnFiveClick");
        if (display_->GetPageIndex() == PageIndex::PAGE_CONFIG) {
            switchTFT();
        }
    });
}

//手工切换网络
XiaoZhiYunliaoS3::BT_STATUS XiaoZhiYunliaoS3::SwitchNetwork() {
    if(GetNetworkType() == NetworkType::ML307){ //4G已开机，则关机重启切换wifi
        SwitchNetworkType();
        return BT_STATUS::SUCCESS;
    }
    Settings settings("board", true);
    int network_type = settings.GetInt("ok_4g", 0); 
    if(network_type == 2){
        SwitchNetworkType();
        return BT_STATUS::SUCCESS;
    }else{
        getPowerManager()->Start4G();
        display_->ShowNotification(Lang::Strings::SWITCH_TO_4G_NETWORK);
        modultype modul_type = check4GModul();
        if (modul_type == modultype::MODUL_4G) {
            settings.SetInt("ok_4g", 2);
            SwitchNetworkType();
            return BT_STATUS::SUCCESS;
        }else{
            display_->ShowNotification(Lang::Strings::SWITCH_TO_WIFI_NETWORK);
            settings.SetInt("ok_4g", 1);
            getPowerManager()->Shutdown4G();
            return BT_STATUS::NO_MODULE;
        }
    }
}

#if CONFIG_USE_BLUETOOTH
void XiaoZhiYunliaoS3::switchBtMode(bool enable) {
    auto& app = Application::GetInstance();
    if (enable) { // 启用蓝牙模式
        if(app.GetAecMode() != kAecOff){
            switchAecMode(kAecOff);
            display_->ShowAEC(false);
        }
        display_->ShowBT(true);
    } else { // 禁用蓝牙模式
        display_->ShowBT(false);
        Settings settings("aec", false);
        int storedMode = settings.GetInt("mode", kAecOnDeviceSide);
        if(storedMode != kAecOff && app.GetAecMode() == kAecOff){
            switchAecMode((AecMode)storedMode);
            display_->ShowAEC(true);
        }
    }
}
#endif

void XiaoZhiYunliaoS3::switchAecMode(AecMode mode) {
    auto& app = Application::GetInstance();
    app.StopListening();
    app.SetDeviceState(kDeviceStateIdle);
    // 直接使用传入的 mode 参数设置 AEC 模式
    app.SetAecMode(mode);
    // 显示通知
    display_->ShowAEC(mode != kAecOff);
    if (mode != kAecOff) {
        display_->ShowNotification(Lang::Strings::RTC_MODE_ON);
    } else {
        display_->ShowNotification(Lang::Strings::RTC_MODE_OFF);
    }
}

void XiaoZhiYunliaoS3::switchTFT() {
    Settings settings("display", true);
    bool currentIpsMode = settings.GetBool("ips_mode", false);
    settings.SetBool("ips_mode", !currentIpsMode);
    ESP_LOGI(TAG, "IPS mode toggled to %s", !currentIpsMode ? "enabled" : "disabled");
    vTaskDelay(pdMS_TO_TICKS(1000));
    auto& app = Application::GetInstance();
    app.Reboot();
}

void XiaoZhiYunliaoS3::switchAecMode(bool enable) {
    AecMode newMode = enable ? kAecOnDeviceSide : kAecOff;
    switchAecMode(newMode);
    Settings settings("aec", true);
    settings.SetInt("mode", newMode);
}

AudioCodec* XiaoZhiYunliaoS3::GetAudioCodec() {
    static Es8388AudioCodec audio_codec(
        codec_i2c_bus_, 
        I2C_NUM_0, 
        AUDIO_INPUT_SAMPLE_RATE, 
        AUDIO_OUTPUT_SAMPLE_RATE,
        AUDIO_I2S_GPIO_MCLK, 
        AUDIO_I2S_GPIO_BCLK, 
        AUDIO_I2S_GPIO_WS, 
        AUDIO_I2S_GPIO_DOUT, 
        AUDIO_I2S_GPIO_DIN,
        AUDIO_CODEC_PA_PIN, 
        AUDIO_CODEC_ES8388_ADDR, 
        AUDIO_INPUT_REFERENCE);
    return &audio_codec;
}

bool XiaoZhiYunliaoS3::GetBatteryLevel(int &level, bool& charging, bool& discharging) {
    level = power_manager_->GetBatteryLevel();
    charging = power_manager_->IsCharging();
    discharging = power_manager_->IsDischarging();
    return true;
}

void XiaoZhiYunliaoS3::Sleep() {
    ESP_LOGI(TAG, "Entering deep sleep");
    Settings settings("board", true);
    settings.SetInt("sleep_flag", 1);

    auto& app = Application::GetInstance();
    app.Close();
    GetBacklight()->SetBrightness(0);
    if (panel) {
        esp_lcd_panel_disp_on_off(panel, false);
    }
    power_manager_->Shutdown4G();
    power_manager_->Shutdown5V();
    power_manager_->MCUSleep();
}



std::string XiaoZhiYunliaoS3::GetHardwareVersion() const {
    std::string version = Lang::Strings::LOGO;
    version += Lang::Strings::VERSION3;
    return version;
}

void XiaoZhiYunliaoS3::SetPowerSaveMode(bool enabled) {
    if (!enabled) {
        power_save_timer_->WakeUp();
    }
    DualNetworkBoard::SetPowerSaveMode(enabled);
}

bool XiaoZhiYunliaoS3::installUart() {
    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    
    if (uart_param_config(uart_num_, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "UART parameter config failed");
        return false;
    }
    
    if (uart_set_pin(uart_num_, ML307_TX_PIN, ML307_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed");
        return false;
    }
    
    if (uart_driver_install(uart_num_, 512, 512, 10, &uart_queue_, 0) != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return false;
    }
    
    isInstallUart_ = true;
    return true;
}

bool XiaoZhiYunliaoS3::uninstallUart() {
    if (uart_driver_delete(uart_num_) != ESP_OK) {
        ESP_LOGE(TAG, "UART driver delete failed");
    }
    
    gpio_reset_pin(ML307_TX_PIN);
    gpio_reset_pin(ML307_RX_PIN);
    
    if (uart_queue_ != nullptr) {
        uart_queue_ = nullptr;
    }
    
    isInstallUart_ = false;
    return true;
}

XiaoZhiYunliaoS3::modultype XiaoZhiYunliaoS3::check4GModul() {
    return checkModuleWithCommand("ATI\r\n", 921600, 10000000);
}

XiaoZhiYunliaoS3::modultype XiaoZhiYunliaoS3::checkModuleWithCommand(const char *command, int baudrate, int64_t timeout) {
    if (!isInstallUart_) {
        if (!installUart()) {
            return MODUL_NONE;
        }
    }
    
    uart_set_baudrate(uart_num_, baudrate);
    ESP_LOGI(TAG, "Sending command: %s", command);
    uart_write_bytes(uart_num_, command, strlen(command));

    int64_t startTime = esp_timer_get_time();
    while ((esp_timer_get_time() - startTime) < timeout) {
        size_t available = 0;
        uart_get_buffered_data_len(uart_num_, &available);

        if (available > 0) {
            int len = uart_read_bytes(uart_num_, (uint8_t *)chbuf_, (available > chbufSize_ - 1) ? chbufSize_ - 1 : available, pdMS_TO_TICKS(10));

            if (len > 0) {
                chbuf_[len] = '\0';

                char *line = strtok(chbuf_, "\n");
                while (line != nullptr) {
                    if (strlen(line) > 1) {
                        if (strncmp(line, "+MATREADY", 9) == 0 ||
                            strncmp(line, "CMCC", 4) == 0 ||
                            strncmp(line, "ML307R", 6) == 0) {
                            int timeout3 = (int)((esp_timer_get_time() - startTime) / 1000000);
                            ESP_LOGI(TAG, "checkModul 4G in %ds:%s", timeout3, chbuf_);
                            return MODUL_4G;
                        }
                    }
                    line = strtok(nullptr, "\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return MODUL_NONE;
}

DECLARE_BOARD(XiaoZhiYunliaoS3);
