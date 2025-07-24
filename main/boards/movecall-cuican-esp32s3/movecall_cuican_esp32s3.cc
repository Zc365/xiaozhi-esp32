#include "movecall_cuican_esp32s3.h"
#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <esp_efuse_table.h>
#include <driver/i2c_master.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_gc9a01.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include <assets/lang_config.h>
#include "power_save_timer.h"
#include <string.h> 
#include <ssid_manager.h>
#include "settings.h"

#define TAG "MovecallCuicanESP32S3"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

void MovecallCuicanESP32S3::InitializeCodecI2c() {
    // Initialize I2C peripheral
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

// SPI初始化
void MovecallCuicanESP32S3::InitializeSpi() {
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(DISPLAY_SPI_SCLK_PIN, DISPLAY_SPI_MOSI_PIN, 
                                DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

// GC9A01初始化
void MovecallCuicanESP32S3::InitializeGc9a01Display() {
    ESP_LOGI(TAG, "Init GC9A01 display");

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(DISPLAY_SPI_CS_PIN, DISPLAY_SPI_DC_PIN, NULL, NULL);
    io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install GC9A01 panel driver");

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = DISPLAY_SPI_RESET_PIN;    // Set to -1 if not use
    panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;           //LCD_RGB_ENDIAN_RGB;
    panel_config.bits_per_pixel = 16;                       // Implemented by LCD command `3Ah` (16/18)

    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true)); 

    display_ = new SpiLcdDisplay(io_handle, panel_handle,
                                DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                {
                                    .text_font = &font_puhui_20_4,
                                    .icon_font = &font_awesome_20_4,
                                    .emoji_font = font_emoji_64_init(),
                                });
}

void MovecallCuicanESP32S3::InitializePowerSaveTimer() {
    power_save_timer_ = new PowerSaveTimer(-1, 120, 600);
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
void MovecallCuicanESP32S3::InitializeButtons() {
    boot_button_.OnClick([this]() {
        ESP_LOGI(TAG, "Button OnClick");
        auto& app = Application::GetInstance();
        std::string wake_word=Lang::Strings::WAKE_WORD;
        app.WakeWordInvoke(wake_word);
    });
    boot_button_.OnPressDown([this]() {
        // ESP_LOGI(TAG, "Button OnPressDown");
        // power_save_timer_->WakeUp();
    });
    boot_button_.OnLongPress([this]() {
        ESP_LOGI(TAG, "Button LongPress to Sleep");
        display_->SetStatus(Lang::Strings::SHUTTING_DOWN);
        vTaskDelay(pdMS_TO_TICKS(2000));
        Sleep();
    });    
    boot_button_.OnFourClick([this]() {
        ESP_LOGI(TAG, "Button OnFourClick");

#if defined(CONFIG_WIFI_FACTORY_SSID)
        if (wifi_config_mode_) {
            auto &ssid_manager = SsidManager::GetInstance();
            auto ssid_list = ssid_manager.GetSsidList();
            if (strlen(CONFIG_WIFI_FACTORY_SSID) > 0){
                ssid_manager.Clear();
                ssid_manager.AddSsid(CONFIG_WIFI_FACTORY_SSID, CONFIG_WIFI_FACTORY_PASSWORD);
                Settings settings("wifi", true);
                settings.SetInt("force_ap", 0);
                esp_restart();
            }
        }else{
            auto &ssid_manager = SsidManager::GetInstance();
            ssid_manager.Clear();
            ESP_LOGI(TAG, "WiFi configuration and SSID list cleared");
            ResetWifiConfiguration();
        }
#else
        auto &ssid_manager = SsidManager::GetInstance();
        ssid_manager.Clear();
        ESP_LOGI(TAG, "WiFi configuration and SSID list cleared");
        ResetWifiConfiguration();

#endif
    });
}

public:
    MovecallCuicanESP32S3() : boot_button_(BOOT_BUTTON_GPIO) {  
        InitializeCodecI2c();
        InitializeSpi();
        InitializeGc9a01Display();
        InitializeButtons();
        GetBacklight()->RestoreBrightness();
    }

Led* MovecallCuicanESP32S3::GetLed() {
    static SingleLed led_strip(BUILTIN_LED_GPIO);
    return &led_strip;
}

Display* MovecallCuicanESP32S3::GetDisplay() {
    return display_;
}

Backlight* MovecallCuicanESP32S3::GetBacklight() {
    static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
    return &backlight;
}

AudioCodec* MovecallCuicanESP32S3::GetAudioCodec() {
    static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
        AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
        AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
    return &audio_codec;
}

DECLARE_BOARD(MovecallCuicanESP32S3);
