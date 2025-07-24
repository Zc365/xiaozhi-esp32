#include "wifi_board.h"
#include "audio/codecs/es8311_audio_codec.h"
#include "xiaoziyunliao_display.h"
#include "xiaozhiyunliao_c3.h"
#include "application.h"
#include "button.h"
#include "settings.h"
#include "config.h"
#include "font_awesome_symbols.h"
#include <wifi_station.h>
#include <ssid_manager.h>
#include <esp_log.h>
// #include "i2c_device.h"
#include <driver/i2c_master.h>
#include <esp_sleep.h>
#include <string.h> 
#include <wifi_configuration_ap.h>
#include <assets/lang_config.h>

#define TAG "YunliaoC3"

#if CONFIG_LCD_CONTROLLER_ILI9341
    #include <esp_lcd_ili9341.h>
    #include <driver/spi_common.h>
#elif CONFIG_LCD_CONTROLLER_ST7789
    #include <esp_lcd_panel_vendor.h>
    #include <driver/spi_common.h>
#endif

    LV_FONT_DECLARE(font_awesome_20_4);
    LV_FONT_DECLARE(font_puhui_20_4);
    #define FONT font_puhui_20_4

esp_lcd_panel_handle_t panel = nullptr;
static QueueHandle_t gpio_evt_queue = NULL;
uint16_t battCnt;//闪灯次数
uint16_t battLife = 0; //电量

// 中断服务程序
static void IRAM_ATTR batt_mon_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// 添加任务处理函数
static void batt_mon_task(void* arg) {
    uint32_t io_num;
    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            battCnt++;
        }
    }
}

static void calBattLife(TimerHandle_t xTimer) {
    // 计算电量
    battLife = battCnt;

    if (battLife > 100){
        battLife = 100;
    }
    // ESP_LOGI(TAG, "Battery num %d\n", (int)battCnt);
    // ESP_LOGI(TAG, "Battery  life %d\n", (int)battLife);
    // 重置计数器
    battCnt = 0;
}

XiaoZhiYunliaoC3::XiaoZhiYunliaoC3() 
    : WifiBoard(),
      boot_button_(BOOT_BUTTON_GPIO, false, KEY_EXPIRE_MS) {  

    Start5V();
    InitializeI2c();
    InitializeButtons();
#if defined(CONFIG_LCD_CONTROLLER_ILI9341) || defined(CONFIG_LCD_CONTROLLER_ST7789)
    InitializeSpi();
    InitializeLCDDisplay();
#endif
    //电量计算定时任务
    InitializeBattMon();
    InitializeBattTimers();
    if(GetAudioCodec()->output_volume() == 0){
        GetAudioCodec()->SetOutputVolume(70);
    }
    if(GetBacklight()->brightness() == 0){
        GetBacklight()->SetBrightness(60);
    }
    InitializePowerSaveTimer();
    ESP_LOGI(TAG, "Inited");
}

void XiaoZhiYunliaoC3::InitializePowerSaveTimer() {
    power_save_timer_ = new PowerSaveTimer(-1, -1, 600);
    power_save_timer_->OnEnterSleepMode([this]() {
        ESP_LOGI(TAG, "Enabling sleep mode");
        auto display = GetDisplay();
        display->SetChatMessage("system", "");
        display->SetEmotion("sleepy");
        GetBacklight()->SetBrightness(10);
        auto codec = GetAudioCodec();
        codec->EnableInput(false);
    });
    power_save_timer_->OnExitSleepMode([this]() {
        auto codec = GetAudioCodec();
        codec->EnableInput(true);
        
        auto display = GetDisplay();
        display->SetChatMessage("system", "");
        display->SetEmotion("neutral");
        GetBacklight()->RestoreBrightness();
    });
    power_save_timer_->OnShutdownRequest([this]() {
        ESP_LOGI(TAG, "Shutting down");
        Sleep();
    });
    power_save_timer_->SetEnabled(true);
}

void XiaoZhiYunliaoC3::PowerSaveTimerSetEnabled(bool enabled) {
    if(power_save_timer_){
        power_save_timer_->SetEnabled(enabled);
    }
}

Backlight* XiaoZhiYunliaoC3::GetBacklight() {
    static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
    return &backlight;
}


void XiaoZhiYunliaoC3::InitializeBattMon() {
    // 电池电量监测引脚配置
    gpio_config_t io_conf_batt_mon = {
        .pin_bit_mask = 1<<PIN_BATT_MON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_batt_mon));
    // 创建电量GPIO事件队列
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // 安装电量GPIO ISR服务
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // 添加中断处理
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_BATT_MON, batt_mon_isr_handler, (void*)PIN_BATT_MON));
     // 创建监控任务
    xTaskCreate(&batt_mon_task, "batt_mon_task", 2048, NULL, 10, NULL);
}

#if defined(CONFIG_LCD_CONTROLLER_ILI9341) || defined(CONFIG_LCD_CONTROLLER_ST7789)
Display* XiaoZhiYunliaoC3::GetDisplay() {
    return display_;
}

void XiaoZhiYunliaoC3::InitializeSpi() {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = DISPLAY_SPI_PIN_MOSI;
    buscfg.miso_io_num = DISPLAY_SPI_PIN_MISO;
    buscfg.sclk_io_num = DISPLAY_SPI_PIN_SCLK;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(DISPLAY_SPI_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}


void XiaoZhiYunliaoC3::InitializeLCDDisplay() {
    esp_lcd_panel_io_handle_t panel_io = nullptr;
    // 液晶屏控制IO初始化
    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = DISPLAY_SPI_PIN_LCD_CS;
    io_config.dc_gpio_num = DISPLAY_SPI_PIN_LCD_DC;
#if CONFIG_LCD_CONTROLLER_ILI9341
    io_config.spi_mode = 0;
#elif CONFIG_LCD_CONTROLLER_ST7789
    io_config.spi_mode = 3;
#endif        
    io_config.pclk_hz = DISPLAY_SPI_CLOCK_HZ;
    io_config.trans_queue_depth = 10;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(DISPLAY_SPI_LCD_HOST, &io_config, &panel_io));

    // 初始化液晶屏驱动芯片
    ESP_LOGD(TAG, "Install LCD driver");
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = DISPLAY_SPI_PIN_LCD_RST;
    panel_config.bits_per_pixel = 16;
    panel_config.rgb_ele_order = DISPLAY_RGB_ORDER_COLOR;
#if CONFIG_LCD_CONTROLLER_ILI9341
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif CONFIG_LCD_CONTROLLER_ST7789
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
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
        DISPLAY_SWAP_XY, 
        {                
            .text_font = &FONT,
            .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
            .emoji_font = font_emoji_32_init(),
#else
            .emoji_font = font_emoji_64_init(),
#endif
        });
        std::string helpMessage = Lang::Strings::HELP4;
         helpMessage += "\n"; 
        helpMessage += Lang::Strings::HELP1;
        helpMessage += "\n"; 
        helpMessage += Lang::Strings::HELP2;
        display_->SetChatMessage("system", helpMessage.c_str());
}
#endif    
void XiaoZhiYunliaoC3::Start5V(){
    gpio_set_level(BOOT_5V_GPIO, 1);
}
void XiaoZhiYunliaoC3::Shutdown5V(){
    gpio_set_level(BOOT_5V_GPIO, 0);
}

void XiaoZhiYunliaoC3::InitializeI2c() {
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
    gpio_config_t io_conf_5v = {
        .pin_bit_mask = 1<<BOOT_5V_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_5v));
}

void XiaoZhiYunliaoC3::InitializeButtons() {
    boot_button_.OnClick([this]() {
        // ESP_LOGI(TAG, "Button OnClick");
        auto& app = Application::GetInstance();
#if CONFIG_USE_ALARM
        app.ToggleChatState();
#endif
        std::string wake_word=Lang::Strings::WAKE_WORD;
        app.WakeWordInvoke(wake_word);
    });
    boot_button_.OnPressDown([this]() {
        // ESP_LOGI(TAG, "Button OnPressDown");
        power_save_timer_->WakeUp();
    });
    boot_button_.OnPressUp([this]() {
        // ESP_LOGI(TAG, "Button OnPressUp");
    });
    boot_button_.OnLongPress([this]() {
        ESP_LOGI(TAG, "Button LongPress to Sleep");
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
    // boot_button_.OnThreeClick([this]() {
    //     ESP_LOGI(TAG, "Button OnThreeClick");
    // });  
    boot_button_.OnFourClick([this]() {
        // ESP_LOGI(TAG, "Button OnFourClick");
        if (display_->GetPageIndex() == PageIndex::PAGE_CONFIG) {
            auto &ssid_manager = SsidManager::GetInstance();
            ssid_manager.Clear();
            ESP_LOGI(TAG, "WiFi configuration and SSID list cleared");
            ResetWifiConfiguration();
            return;
        }
#if defined(CONFIG_WIFI_FACTORY_SSID)
        if (wifi_config_mode_) {
            auto &ssid_manager = SsidManager::GetInstance();
            auto ssid_list = ssid_manager.GetSsidList();
            if (strlen(CONFIG_WIFI_FACTORY_SSID) > 0){
                ESP_LOGI(TAG, "Factory SSID %s set", CONFIG_WIFI_FACTORY_SSID);
                ssid_manager.Clear();
                ssid_manager.AddSsid(CONFIG_WIFI_FACTORY_SSID, CONFIG_WIFI_FACTORY_PASSWORD);
                Settings settings("wifi", true);
                settings.SetInt("force_ap", 0);
                esp_restart();
            }
        }
#endif
    });
}

void XiaoZhiYunliaoC3::InitializeBattTimers() {
    batt_ticker_ = xTimerCreate("BattTicker", pdMS_TO_TICKS(62500), pdTRUE, (void*)0, calBattLife);
    if (batt_ticker_ != NULL) {
        xTimerStart(batt_ticker_, 0);
    }
}

AudioCodec* XiaoZhiYunliaoC3::GetAudioCodec() {
    static Es8311AudioCodec audio_codec(
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
        AUDIO_CODEC_ES8311_ADDR);
    return &audio_codec;
}

bool XiaoZhiYunliaoC3::GetBatteryLevel(int &level, bool& charging, bool& discharging) {
    level = battLife;
    charging = false;
    discharging = false;
    return true;
}

void XiaoZhiYunliaoC3::EnterWifiConfigMode() {
    auto& application = Application::GetInstance();
    application.SetDeviceState(kDeviceStateWifiConfiguring);

    auto& wifi_ap = WifiConfigurationAp::GetInstance();
    wifi_ap.SetLanguage(Lang::CODE);
    wifi_ap.SetSsidPrefix("Xiaozhi");
    wifi_ap.Start();
    wifi_ap.StartSmartConfig();

#if (defined ja_jp) || (defined en_us)
    std::string hint = "";
#else
    std::string hint = Lang::Strings::SCAN_QR;
#endif
    hint += Lang::Strings::CONNECT_TO_HOTSPOT;
    hint += wifi_ap.GetSsid();
    hint += Lang::Strings::ACCESS_VIA_BROWSER;
    hint += wifi_ap.GetWebServerUrl();
    hint += Lang::Strings::HINT_SHUTDOWN;
    
    application.Alert(Lang::Strings::WIFI_CONFIG_MODE, hint.c_str(), "system", Lang::Sounds::P3_WIFICONFIG);

    auto display = Board::GetInstance().GetDisplay();
    static_cast<XiaoziyunliaoDisplay*>(display)->NewSmartConfigPage();
    
    while (true) {
        int free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        int min_free_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "Free internal: %u minimal internal: %u", free_sram, min_free_sram);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void XiaoZhiYunliaoC3::Sleep() {
    ESP_LOGI(TAG, "Entering deep sleep");

    Application::GetInstance().StopListening();
    if (auto* codec = GetAudioCodec()) {
        codec->EnableOutput(false);
        codec->EnableInput(false);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_remove(PIN_BATT_MON));
    if(gpio_evt_queue) {
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
    }
    GetBacklight()->SetBrightness(0);
    Shutdown5V();
    if (panel) {
        esp_lcd_panel_disp_on_off(panel, false);
    }
    MCUSleep();
}

void XiaoZhiYunliaoC3::MCUSleep() {
    gpio_deep_sleep_hold_dis();
    esp_deep_sleep_enable_gpio_wakeup(0b0010, ESP_GPIO_WAKEUP_GPIO_LOW);
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);
    esp_deep_sleep_start();
}

void XiaoZhiYunliaoC3::StopNetwork() {
    if (wifi_config_mode_) {
        auto& wifi_ap = WifiConfigurationAp::GetInstance();
        wifi_ap.Stop();
    } else {
        auto& wifi_station = WifiStation::GetInstance();
        wifi_station.Stop();
    }
    auto display = Board::GetInstance().GetDisplay();
    display->ShowNotification(Lang::Strings::DISCONNECT);
}


std::string XiaoZhiYunliaoC3::GetHardwareVersion() const {
    std::string version = Lang::Strings::LOGO;
#if CONFIG_BOARD_TYPE_YUNLIAO_V1
    version += Lang::Strings::VERSION1;
#else
    version += Lang::Strings::VERSION2;
#endif
    return version;
}

DECLARE_BOARD(XiaoZhiYunliaoC3);
