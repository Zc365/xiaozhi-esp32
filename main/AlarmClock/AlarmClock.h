#ifndef ALARMCLOCK_H
#define ALARMCLOCK_H

#include <string>
#include <esp_log.h>
#include <esp_timer.h>
#include "time.h"
#include <mutex>
#include "settings.h"
#include <atomic>
#if CONFIG_USE_ALARM
struct Alarm {
    std::string name;
    int time;
};

class AlarmManager {
public:
    AlarmManager();
    ~AlarmManager();

    // 设置闹钟
    std::string SetAlarm(int seconde_from_now, std::string alarm_name);
    //检查闹钟配置
    bool CheckAlarmConfig(time_t now, bool printLog = false);
    // 闹钟响了的处理函数
    void CheckAlarms(time_t now);
    // 闹钟是不是响了的标志位
    bool IsRing(){ return ring_flag; };
    // 清除闹钟标志位
    void ClearRing();
    //响铃倒计时
    int DelaySecond;

private:
    std::mutex mutex_; // 互斥锁
    std::atomic<bool> ring_flag{false}; 
    Alarm first_Alarm; // 缓存最近的闹钟
};
#endif
#endif