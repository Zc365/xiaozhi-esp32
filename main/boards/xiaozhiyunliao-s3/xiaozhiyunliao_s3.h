#ifndef XIAOZHIYUNLIAO_S3_H
#define XIAOZHIYUNLIAO_S3_H
#include "application.h"
#include "dual_network_board.h"
// #include "codecs/es8311_audio_codec.h"
#include "button.h"
// #include "driver/gpio.h"
// #include "driver/i2c_master.h"
// #include "esp_log.h"
#include "xiaoziyunliao_display.h"
#include "power_save_timer.h"
#include "power_manager.h"

// class XiaoziyunliaoDisplay;

class XiaoZhiYunliaoS3 : public DualNetworkBoard {
private:
    enum modultype { MODUL_NONE = 0, MODUL_BT = 1, MODUL_4G = 2 };

    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    
    XiaoziyunliaoDisplay* display_;
    
    uart_port_t uart_num_;
    QueueHandle_t uart_queue_;
    bool isInstallUart_;
    int is4Ginstalled;
    char *chbuf_;
    const uint16_t chbufSize_ = 64;
    
    void InitializeSpi();
    void InitializeLCDDisplay();
    void InitializeI2c();
    void InitializeButtons();
    void InitializePowerSaveTimer();
    void InitializeBTEmitter();
    void InitializeModul();
    
    modultype check4GModul();
    modultype checkModuleWithCommand(const char *command, int baudrate, int64_t timeout);
    bool installUart();
    bool uninstallUart();
    
    static void gpioTask(void *arg);
    void deinitBTEmitter();
    
    TaskHandle_t m_gpio_task_handle = nullptr;
public:
    enum class BT_STATUS {
        SUCCESS,            // 操作成功
        ALREADY_STARTED,    // 蓝牙已开启
        ALREADY_STOPPED,    // 蓝牙已关闭
        NO_MODULE,       // 未安装蓝牙模块
    };

    XiaoZhiYunliaoS3();
    virtual ~XiaoZhiYunliaoS3() = default;

    virtual Display* GetDisplay() override;
    virtual Backlight* GetBacklight() override;
    virtual AudioCodec* GetAudioCodec() override;
    bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override;
    void Sleep();
    void SetPressToTalkEnabled(bool enabled);
    std::string GetHardwareVersion() const override;
    void PowerSaveTimerSetEnabled(bool enabled);
    virtual void SetPowerSaveMode(bool enabled) override;
    PowerManager* getPowerManager(){ return power_manager_; };
    void switchAecMode(AecMode mode);
    void switchAecMode(bool enable);
    void switchBtMode(bool enable);
    void switchTFT();
    XiaoZhiYunliaoS3::BT_STATUS SwitchNetwork();
    BT_STATUS SwitchBluetooth(bool switch_on);
};

#endif // XIAOZHIYUNLIAO_S3_H