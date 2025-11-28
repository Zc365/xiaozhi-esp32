#ifndef XIAOZHIYUNLIAO_S3_H
#define XIAOZHIYUNLIAO_S3_H
#include "application.h"
#include "dual_network_board.h"
#include "codecs/es8311_audio_codec.h"
#include "button.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "xiaoziyunliao_display.h"
#include "power_save_timer.h"
#include "power_manager.h"
#if CONFIG_USE_BLUETOOTH
    #include "BT_Emitter.h"
#endif

// class XiaoziyunliaoDisplay;

class XiaoZhiYunliaoS3 : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
#if CONFIG_USE_BLUETOOTH
    BT_Emitter* bt_emitter_;
#endif
    
    XiaoziyunliaoDisplay* display_;
    void InitializeSpi();
    void InitializeLCDDisplay();
    void InitializeI2c();
    void InitializeButtons();
    void InitializePowerSaveTimer();

public:
    enum class BT_STATUS {
        SUCCESS,            // 操作成功
        ALREADY_STARTED,    // 蓝牙已开启
        ALREADY_STOPPED,    // 蓝牙已关闭
        NO_BT_MODULE,       // 未安装蓝牙模块
    };

    XiaoZhiYunliaoS3();
    virtual ~XiaoZhiYunliaoS3() = default;

    virtual Display* GetDisplay() override;
    virtual Backlight* GetBacklight() override;
    virtual AudioCodec* GetAudioCodec() override;
    bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override;
    void EnterWifiConfigMode() override;
    void Sleep();
    void SetPressToTalkEnabled(bool enabled);
    std::string GetHardwareVersion() const override;
    void PowerSaveTimerSetEnabled(bool enabled);
    virtual void SetPowerSaveMode(bool enabled) override;
    PowerManager* getPowerManager(){ return power_manager_; };
    void switchAecMode(AecMode mode);
    void switchBtMode(bool enable);
#if CONFIG_USE_BLUETOOTH
    BT_Emitter* GetBTEmitter(){ return bt_emitter_; } ;
    BT_STATUS SwitchBluetooth(bool switch_on);
#endif
};

#endif // XIAOZHIYUNLIAO_S3_H