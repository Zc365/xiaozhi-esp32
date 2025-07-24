#ifndef MogaiCuicanESP32S3_H
#define MogaiCuicanESP32S3_H

#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "mogai_cuican_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
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

class MogaiCuicanESP32S3 : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    MogaiCuicanDisplay* display_;
    esp_lcd_panel_handle_t panel_handle = NULL;
    PowerSaveTimer* power_save_timer_;

    void InitializeCodecI2c();
    void InitializeSpi();
    void InitializeGc9a01Display();
    void InitializePowerSaveTimer();
    void InitializeButtons();
    void InitializeIot();
    void MCUSleep();

public:
    MogaiCuicanESP32S3();
    void Sleep();
    virtual ~MogaiCuicanESP32S3() = default;
    virtual Led* GetLed() override;
    virtual Display* GetDisplay() override ;
    virtual Backlight* GetBacklight() override ;
    virtual AudioCodec* GetAudioCodec() override ;
    void EnterWifiConfigMode() override;
};

#endif 