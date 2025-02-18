#include "board.h"
#include "boards/common/wifi_board.h"
#include "iot/thing.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "application.h"
#include "esp_system.h"
#include "system_info.h"
#define TAG "BoardControl"

namespace iot {

class BoardControl : public Thing {
private:
    TimerHandle_t sleep_timer_;
    enum class TimerMode { SLEEP, RESTART }; 
    TimerMode timer_mode_; 

    // 固件版本属性获取方法
    std::string GetFirmwareVersion() const {
        return Application::GetInstance().getOta().GetFirmwareVersion();
    }

    std::string GetCurrentVersion() const {
        return Application::GetInstance().getOta().GetCurrentVersion();
    }

    bool HasNewVersion() const {
        return Application::GetInstance().getOta().HasNewVersion();
    }

    // 定时器回调函数
    static void SleepTimerCallback(TimerHandle_t xTimer) {
        BoardControl* instance = static_cast<BoardControl*>(pvTimerGetTimerID(xTimer));
        if (instance->timer_mode_ == TimerMode::SLEEP) {
            ESP_LOGI(TAG, "System entering sleep mode after delay");
            Board::GetInstance().Sleep();
        } else {
            ESP_LOGI(TAG, "System restarting after delay");
            esp_restart();  // 添加重启逻辑
        }
    }

public:
    BoardControl() : Thing("BoardControl", "当前 AI 机器人管理和控制"), timer_mode_(TimerMode::SLEEP) {
        // 创建定时器时传递this指针作为ID
        sleep_timer_ = xTimerCreate("SleepTimer", pdMS_TO_TICKS(5000), pdFALSE, this, SleepTimerCallback);
        
        // 添加电池电量属性
        properties_.AddNumberProperty("BatteryLevel", "当前电池电量百分比", [this]() -> int {
            int level = 0;
            bool charging = false;
            Board::GetInstance().GetBatteryLevel(level, charging);
            // ESP_LOGI(TAG, "当前电池电量: %d%%, 充电状态: %s", level, charging ? "充电中" : "未充电");
            return level;
        });

        // 添加充电状态属性
        properties_.AddBooleanProperty("Charging", "是否正在充电", [this]() -> bool {
            int level = 0;
            bool charging = false;
            Board::GetInstance().GetBatteryLevel(level, charging);
            // ESP_LOGI(TAG, "当前电池电量: %d%%, 充电状态: %s", level, charging ? "充电中" : "未充电");
            return charging;
        });

        properties_.AddStringProperty("FirmwareVersion", "最新固件版本", 
            [this]() -> std::string { return GetFirmwareVersion(); });

        properties_.AddStringProperty("CurrentVersion", "当前固件版本",
            [this]() -> std::string { return GetCurrentVersion(); });

        properties_.AddBooleanProperty("HasNewVersion", "是否有新固件版本",
            [this]() -> bool { return HasNewVersion(); });

        properties_.AddStringProperty("MACAddress", "设备MAC地址",
            []() -> std::string { return SystemInfo::GetMacAddress(); });

        properties_.AddNumberProperty("FlashSize", "闪存容量(MB)",
            []() -> int { return SystemInfo::GetFlashSize() / 1024 / 1024; });

        properties_.AddStringProperty("ChipModel", "芯片型号",
            []() -> std::string { return SystemInfo::GetChipModelName(); });

        properties_.AddNumberProperty("FreeHeap", "可用内存(bytes)",
            []() -> int { return SystemInfo::GetFreeHeapSize(); });

        // 修改休眠方法
        methods_.AddMethod("Sleep", "延迟5秒后进入关机/休眠状态", ParameterList(), 
            [this](const ParameterList& parameters) {
                ESP_LOGI(TAG, "Delaying sleep for 5 seconds");
                if (sleep_timer_ != NULL) {
                    timer_mode_ = TimerMode::SLEEP;  // 设置模式为休眠
                    xTimerStart(sleep_timer_, 0);
                }
            });

        // 重启方法
        methods_.AddMethod("Restart", "延迟5秒后重启设备", ParameterList(),
            [this](const ParameterList& parameters) {
                ESP_LOGI(TAG, "Delaying restart for 5 seconds");
                if (sleep_timer_ != NULL) {
                    timer_mode_ = TimerMode::RESTART;  // 设置模式为重启
                    xTimerStart(sleep_timer_, 0);
                }
            });

        // 修改重新配网
        methods_.AddMethod("ResetWifiConfiguration", "重新配网", ParameterList(), 
            [this](const ParameterList& parameters) {
                ESP_LOGI(TAG, "ResetWifiConfiguration");
                auto board = static_cast<WifiBoard*>(&Board::GetInstance());
                if (board && board->GetBoardType() == "wifi") {
                    board->ResetWifiConfiguration();
                }
            });

        // 固件更新方法
        methods_.AddMethod("UpdateFirmware", "立即更新固件", ParameterList(),
            [this](const ParameterList& parameters) -> bool {
                return Application::GetInstance().UpdateNewVersion();
            });

    }
    
    // 添加析构函数释放定时器资源
    ~BoardControl() {
        if (sleep_timer_ != NULL) {
            xTimerDelete(sleep_timer_, 0);
        }
    }
};

} // namespace iot

DECLARE_THING(BoardControl); 