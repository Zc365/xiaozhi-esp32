#include "board.h"
#include "boards/common/wifi_board.h"
#include "iot/thing.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "application.h"
#define TAG "BoardControl"

namespace iot {

class BoardControl : public Thing {
private:
    TimerHandle_t sleep_timer_;  // 添加定时器句柄

    // 新增固件版本属性获取方法
    std::string GetFirmwareVersion() const {
        return Application::GetInstance().getOta().GetFirmwareVersion();
    }

    std::string GetCurrentVersion() const {
        return Application::GetInstance().getOta().GetCurrentVersion();
    }

    bool HasNewVersion() const {
        return Application::GetInstance().getOta().HasNewVersion();
    }

    // 添加定时器回调函数
    static void SleepTimerCallback(TimerHandle_t xTimer) {
        ESP_LOGI(TAG, "System entering sleep mode after delay");
        Board::GetInstance().Sleep();
    }

public:
    BoardControl() : Thing("BoardControl", "当前 AI 机器人管理和控制") {
        // 创建单次定时器
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

        // 添加固件版本属性
        properties_.AddStringProperty("FirmwareVersion", "最新固件版本", 
            [this]() -> std::string { return GetFirmwareVersion(); });

        properties_.AddStringProperty("CurrentVersion", "当前固件版本",
            [this]() -> std::string { return GetCurrentVersion(); });

        properties_.AddBooleanProperty("HasNewVersion", "是否有新固件版本",
            [this]() -> bool { return HasNewVersion(); });

        // 修改休眠方法
        methods_.AddMethod("Sleep", "进入关机/休眠状态", ParameterList(), 
            [this](const ParameterList& parameters) {
                ESP_LOGI(TAG, "Delaying sleep for 5 seconds");
                if (sleep_timer_ != NULL) {
                    xTimerStart(sleep_timer_, 0);  // 启动定时器
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

        // 修改固件更新方法添加返回值
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