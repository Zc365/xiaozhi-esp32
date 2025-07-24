#ifndef __POWERMANAGER_H__
#define __POWERMANAGER_H__

#include <functional>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

class PowerManager{
public:
    PowerManager();
    void Initialize();
    bool IsCharging();
    bool IsDischarging();
    bool IsChargingDone();
    int GetBatteryLevel();
    void Start5V();
    void Shutdown5V();
    void MCUSleep();
    void CheckBatteryStatus();
    void OnChargingStatusChanged(std::function<void(bool)> callback);
    void OnChargingStatusDisChanged(std::function<void(bool)> callback);
private:
    esp_timer_handle_t timer_handle_;
    std::function<void(bool)> charging_callback_;
    std::function<void(bool)> discharging_callback_;
    bool is_charging_ = false;
    bool is_discharging_ = true;
    int call_count_ = 0;
};

#endif