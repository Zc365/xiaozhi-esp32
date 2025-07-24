#include "AlarmClock.h"
#include "assets/lang_config.h"
#include "board.h"
#include "display.h"
#include "application.h"
#define TAG "AlarmManager"

#if CONFIG_USE_ALARM


bool AlarmManager::CheckAlarmConfig(time_t now, bool printLog) {
    first_Alarm = {"", 0}; // 重置缓存
    bool found_first = false;
    Settings settings_("alarm_clock", true);
    for(int i = 0; i < 10; i++) {
        std::string alarm_name = settings_.GetString("alarm_" + std::to_string(i));
        if(!alarm_name.empty()){
            int alarm_time = settings_.GetInt("alarm_time_" + std::to_string(i));
            if(alarm_time == 0 || alarm_time <= now ){ //过期不合理记录或记录清除
                settings_.SetString("alarm_" + std::to_string(i), "");
                settings_.SetInt("alarm_time_" + std::to_string(i), 0);
                if(printLog){
                    int time_diff2 = now - alarm_time;
                    ESP_LOGI(TAG, "%d.Alarm:%s overdue %ds is clean", i, alarm_name.c_str(), time_diff2);
                }
            }else if(alarm_time > now){ //有效记录取得最新一条
                if(!found_first || alarm_time < first_Alarm.time) {
                    first_Alarm.name = alarm_name;
                    first_Alarm.time = alarm_time;
                    found_first = true;
                }
                if(printLog){
                    int time_diff = alarm_time - now;
                    ESP_LOGI(TAG, "%d.Alarm:%s after %ds ok", i, alarm_name.c_str(), time_diff);
                }
            }
        }
    }
    return found_first;
}

AlarmManager::AlarmManager() {
    ESP_LOGI(TAG, "AlarmManager init");
    ring_flag = false;
    CheckAlarmConfig(time(NULL), true);
}

AlarmManager::~AlarmManager() {
}

void AlarmManager::CheckAlarms(time_t now) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool hasTriggered = false;
    if(!first_Alarm.name.empty() && first_Alarm.time != 0){
        int exceed = now - first_Alarm.time;
        if(exceed >= 0) {
            ring_flag = true;
            hasTriggered = true;
            auto &app = Application::GetInstance();
            app.SetAlarmEvent();
            auto display = Board::GetInstance().GetDisplay();
            display->SetStatus(first_Alarm.name.c_str());
            ESP_LOGI(TAG, "Alarm:%s triggered, exceed %ds", first_Alarm.name.c_str(), exceed);
        }        
    }
    if (hasTriggered) {
        CheckAlarmConfig(now,true);
    }
}

std::string AlarmManager::SetAlarm(int seconde_from_now, std::string alarm_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    Settings settings_("alarm_clock", true);
    
    // 验证时间参数
    if(seconde_from_now <= 0 || seconde_from_now > 31*24*3600) {
        ESP_LOGE(TAG, "Invalid alarm time");
        return "{\"success\": false, \"message\": \"Invalid alarm time\"}";
    }
    
    // 寻找空闲位置存储闹钟 
    int free_index = -1;
    for(int i = 0; i < 10; i++) {
        if(settings_.GetString("alarm_" + std::to_string(i)).empty()) {
            free_index = i;
            break;
        }
    }
    
    if(free_index == -1) {
        ESP_LOGE(TAG, "too many alarms");
        return "{\"success\": false, \"message\": \"too many alarms\"}";
    }
    
    // 存储新闹钟
    time_t alarm_time = time(NULL) + seconde_from_now;
    settings_.SetString("alarm_" + std::to_string(free_index), alarm_name);
    settings_.SetInt("alarm_time_" + std::to_string(free_index), alarm_time);
    ESP_LOGI(TAG, "%d.Alarm:%s (%ds) set ok", free_index, alarm_name.c_str(), seconde_from_now);
    
    // 更新缓存
    time_t now = time(NULL);
    CheckAlarmConfig(now);
    
    // 生成响应信息
    std::string status;
    int cnt = 0;
    for(int i = 0; i < 10; i++) {
        std::string name = settings_.GetString("alarm_" + std::to_string(i));
        int t = settings_.GetInt("alarm_time_" + std::to_string(i));
        if(!name.empty() && t > 0) {
            status += name + " after " + std::to_string(t - now) + "seconds\n";
            cnt++;
        }
    }
    std::string status_json = "{\"success\": true, \"count\": " + std::to_string(cnt) + 
           ", \"alarms\": \"" + status + "\"}";
    ESP_LOGI(TAG, "SetAlarm : %s", status_json.c_str());
    return status_json;
}

void AlarmManager::ClearRing() {
    ring_flag = false;
    auto &app = Application::GetInstance();
    app.ClearAlarmEvent();
    ESP_LOGI(TAG, "Alarm cleared");
}
#endif
