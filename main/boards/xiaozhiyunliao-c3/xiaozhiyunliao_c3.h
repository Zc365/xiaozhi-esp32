#ifndef XIAOZHIYUNLIAO_C3_H
#define XIAOZHIYUNLIAO_C3_H

#include "wifi_board.h"
#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "xiaoziyunliao_display.h"
#include "power_save_timer.h"

class XiaoziyunliaoDisplay;

class XiaoZhiYunliaoC3 : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    TimerHandle_t batt_ticker_;
    PowerSaveTimer* power_save_timer_;
    
    XiaoziyunliaoDisplay* display_;
    
    void InitializeSpi();
    void InitializeLCDDisplay();
    void InitializeI2c();
    void InitializeButtons();
    void InitializeBattMon();
    void InitializeBattTimers();
    void Start5V();
    void Shutdown5V();
    void InitializePowerSaveTimer();

public:
    XiaoZhiYunliaoC3();
    virtual ~XiaoZhiYunliaoC3() = default;

    virtual Display* GetDisplay() override;
    virtual Backlight* GetBacklight() override;
    virtual AudioCodec* GetAudioCodec() override;
    bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override;
    void EnterWifiConfigMode() override;
    void Sleep();
    void StopNetwork();
    void SetPressToTalkEnabled(bool enabled);
    std::string GetHardwareVersion() const override;
    void MCUSleep();
    void PowerSaveTimerSetEnabled(bool enabled);
};

#endif // XIAOZHIYUNLIAO_C3_H 