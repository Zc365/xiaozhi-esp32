#include "xiaoziyunliao_display.h"
#include "xiaozhiyunliao_s3.h"
#include "assets/lang_config.h"
#include <font_awesome.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/ledc.h>
#include <vector>
#include <esp_lvgl_port.h>
#include "board.h"
#include <string.h>
#include "settings.h"
#include "lvgl_theme.h"
#if CONFIG_USE_WEATHER
    #include "weather_forecast.h"
#endif

#define TAG "YunliaoDisplay"

LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(font_num_70_2);
// 前向声明天气字体函数
extern "C" {
const lv_font_t* font_weather_32(void);
}

// #if defined(ja_jp)
//     LV_FONT_DECLARE(font_noto_14_1_ja_jp);
//     #define FONT_CONF &font_noto_14_1_ja_jp
// #elif defined(en_us)
//     LV_FONT_DECLARE(font_puhui_14_1);
//     #define FONT_CONF &font_puhui_14_1
// #else
//     #define FONT_CONF fonts_.text_font
// #endif

constexpr char CONSOLE_URL[] = "https://xiaozhi.me/console";
constexpr char WIFI_URL[] = "https://iot.espressif.cn/configWXDeviceWiFi.html";

#if CONFIG_USE_WEATHER
    static void weather_code_to_utf8(uint16_t code, char* buf) {
        uint32_t unicode = 0xE000 + code;
        // 转换为UTF-8编码 (三字节格式)
        buf[0] = 0xE0 | ((unicode >> 12) & 0x0F); // 1110xxxx
        buf[1] = 0x80 | ((unicode >> 6) & 0x3F);  // 10xxxxxx
        buf[2] = 0x80 | (unicode & 0x3F);         // 10xxxxxx
        buf[3] = '\0';
    }
#endif

XiaoziyunliaoDisplay::XiaoziyunliaoDisplay(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_handle_t panel,
    gpio_num_t backlight_pin,
    bool backlight_output_invert,
    int width, int height,
    int offset_x, int offset_y,
    bool mirror_x, bool mirror_y,
    bool swap_xy)
    : SpiLcdDisplay(panel_io, panel,
                width, height,
                offset_x, offset_y,
                mirror_x, mirror_y,
                swap_xy){
        SetupUI();
}

void XiaoziyunliaoDisplay::SetupUI() {
    LvglTheme* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    auto text_font = lvgl_theme->text_font()->font();
    
    DisplayLockGuard lock(this);
    LcdDisplay::SetupUI();

    // 创建tabview，填充整个屏幕
    tabview_ = lv_tabview_create(lv_scr_act());
    lv_obj_set_size(tabview_, LV_HOR_RES, LV_VER_RES - text_font->line_height);
    lv_obj_align_to(tabview_, status_bar_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);  // 紧贴
    // 隐藏标签栏
    lv_tabview_set_tab_bar_position(tabview_, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tabview_, 0);
    lv_obj_t * tab_btns = lv_tabview_get_tab_btns(tabview_);
    lv_obj_add_flag(tab_btns, LV_OBJ_FLAG_HIDDEN);
    // 设置tabview的滚动捕捉模式，确保滑动后停留在固定位置
    lv_obj_t * content = lv_tabview_get_content(tabview_);
    lv_obj_set_scroll_snap_x(content, LV_SCROLL_SNAP_CENTER);
    
    // 创建两个TAB页面
    tab_main = lv_tabview_add_tab(tabview_, "TabMain");
    tab_idle = lv_tabview_add_tab(tabview_, "TabIdle");
    lv_obj_remove_flag(tab_main, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(tab_main, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(tab_idle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(tab_idle, LV_SCROLLBAR_MODE_OFF);

    SetupTabMain();
    SetupTabIdle();

}

void XiaoziyunliaoDisplay::SetupTabMain() {
    LvglTheme* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    DisplayLockGuard lock(this);

    lv_obj_set_style_text_font(tab_main, lvgl_theme->text_font()->font(), 0);
    lv_obj_set_style_text_color(tab_main, lvgl_theme->text_color(), 0);
    lv_obj_set_style_bg_color(tab_main, lvgl_theme->background_color(), 0);
    lv_obj_set_style_bg_opa(tab_main, LV_OPA_COVER, 0); 


    /* Container */
    chat_container_ = lv_obj_create(tab_main);
    lv_obj_set_size(chat_container_, LV_HOR_RES, LV_VER_RES - lvgl_theme->text_font()->font()->line_height);
    lv_obj_align_to(tabview_, status_bar_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0); 
    lv_obj_set_pos(chat_container_, -13, -13);
    lv_obj_set_flex_flow(chat_container_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(chat_container_, 0, 0);
    lv_obj_set_style_border_width(chat_container_, 0, 0);
    lv_obj_set_style_pad_row(chat_container_, 0, 0);
    lv_obj_set_style_text_color(chat_container_, lvgl_theme->text_color(), 0);
    lv_obj_set_style_bg_color(chat_container_, lvgl_theme->background_color(), 0);

    /* Content */
    content_ = lv_obj_create(chat_container_);
    lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_radius(content_, 0, 0);
    lv_obj_set_width(content_, LV_HOR_RES);
    lv_obj_set_flex_grow(content_, 1);
    lv_obj_set_style_pad_all(content_, 0, 0);
    lv_obj_set_style_border_width(content_, 0, 0);
    lv_obj_set_style_text_color(content_, lvgl_theme->text_color(), 0);
    lv_obj_set_style_bg_color(content_, lvgl_theme->background_color(), 0);

    lv_obj_set_flex_flow(content_, LV_FLEX_FLOW_COLUMN); // 垂直布局（从上到下）
    lv_obj_set_flex_align(content_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY); // 子对象居中对齐，等距分布

    NewChatPage();
}

void XiaoziyunliaoDisplay::SetupTabIdle() {
    LvglTheme* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    DisplayLockGuard lock(this);

    lv_obj_set_style_text_font(tab_idle, lvgl_theme->text_font()->font(), 0);
    lv_obj_set_style_text_color(tab_idle, lvgl_theme->text_color(), 0);
    lv_obj_set_style_bg_color(tab_idle, lvgl_theme->background_color(), 0);
    lv_obj_set_style_bg_opa(tab_idle, LV_OPA_COVER, 0); 
  
    //日期标签
    date_label_ = lv_label_create(tab_idle);
    lv_label_set_text(date_label_, "");//10月1日
    lv_obj_align(date_label_, LV_ALIGN_TOP_MID, -60, 0);
    
    // 星期标签
    weekday_label_ = lv_label_create(tab_idle);
    lv_label_set_text(weekday_label_, "");//星期一
#if CONFIG_USE_WEATHER
    lv_obj_align(weekday_label_, LV_ALIGN_TOP_MID, -60, 25);
#else
    lv_obj_align(weekday_label_, LV_ALIGN_TOP_MID, 60, 0);
#endif
#if CONFIG_USE_WEATHER    
    //城市标签
    city_label_ = lv_label_create(tab_idle);
    lv_label_set_text(city_label_, "读取中");//北京
    lv_obj_align(city_label_, LV_ALIGN_TOP_MID, 60, 0);

    //空气标签
    aqi_label_ = lv_label_create(tab_idle);
    lv_obj_set_style_text_color(aqi_label_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(aqi_label_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(aqi_label_, lv_color_hex(0x18EB4A), 0);
    lv_obj_set_style_radius(aqi_label_, 3, 0);
    lv_obj_set_style_pad_all(aqi_label_, 1, 0);  // 增加内边距
    lv_obj_set_style_min_width(aqi_label_, 70, 0); 
    lv_obj_set_style_text_align(aqi_label_, LV_TEXT_ALIGN_CENTER, 0);// 设置文本在标签内水平居中
    lv_label_set_text(aqi_label_, "空气优");
    lv_obj_align(aqi_label_, LV_ALIGN_TOP_MID, 60, 25);
#endif
    // 时间标签容器
    lv_obj_t* time_container_ = lv_obj_create(tab_idle);
    lv_obj_remove_style_all(time_container_);  // 移除所有默认样式
    lv_obj_set_size(time_container_, LV_SIZE_CONTENT, LV_SIZE_CONTENT); // 大小根据内容调整
    lv_obj_set_style_pad_all(time_container_, 0, 0); // 无内边距
    lv_obj_set_style_bg_opa(time_container_, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_width(time_container_, 0, 0); // 无边框
    lv_obj_set_flex_flow(time_container_, LV_FLEX_FLOW_ROW);// 设置为水平Flex布局
    lv_obj_set_flex_align(time_container_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_align(time_container_, LV_ALIGN_CENTER, 0, -5);// 设置容器位置为屏幕中央
    
    // 创建小时标签
    hour_label_ = lv_label_create(time_container_);
    lv_obj_set_style_text_font(hour_label_, &font_num_70_2, 0);
    lv_label_set_text(hour_label_, "00 ");

    // 创建冒号标签
    colon_label_ = lv_label_create(time_container_);
    lv_obj_set_style_text_font(colon_label_, &font_num_70_2, 0);
    lv_label_set_text(colon_label_, ":");
    lv_obj_set_style_translate_y(colon_label_, -5, 0); // 上移5像素

    // 创建分钟标签，使用橙色显示
    minute_label_ = lv_label_create(time_container_);
    lv_obj_set_style_text_font(minute_label_, &font_num_70_2, 0);
    lv_obj_set_style_text_color(minute_label_, lv_color_hex(0xF1BA3B), 0); // 橙色
    lv_label_set_text(minute_label_, "00");
#if CONFIG_USE_WEATHER
    // 创建天气类型标签
    weather_label_ = lv_label_create(tab_idle);
    lv_obj_set_style_text_color(weather_label_, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(weather_label_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(weather_label_, lv_color_hex(0xF1BA3B), 0);
    lv_obj_set_style_radius(weather_label_, 3, 0);
    lv_obj_set_style_pad_all(weather_label_, 1, 0);  // 增加内边距
    lv_obj_set_style_min_width(weather_label_, 70, 0); 
    lv_obj_set_style_text_align(weather_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(weather_label_, "晴");
    lv_obj_align(weather_label_, LV_ALIGN_BOTTOM_MID, -60, -40);

    //风向标签
    wind_label_ = lv_label_create(tab_idle);
    lv_label_set_text(wind_label_, "");//西南风二级
    lv_obj_align(wind_label_, LV_ALIGN_BOTTOM_MID, -60, -15);

    // 温度标签
    char icon_buf[4];
    temperature_label1_ = lv_label_create(tab_idle);
    const lv_font_t* weather_font = font_weather_32();
    if (weather_font != NULL) {
        lv_obj_set_style_text_font(temperature_label1_, weather_font, LV_PART_MAIN);
    } else {
        ESP_LOGE(TAG, "Failed to get weather font");
    }
    lv_obj_align(temperature_label1_, LV_ALIGN_BOTTOM_MID, 10, -35);
    weather_code_to_utf8(900, icon_buf);
    lv_label_set_text(temperature_label1_, icon_buf);

    temperature_label2_ = lv_label_create(tab_idle);
    lv_label_set_text(temperature_label2_, "");//28℃
    lv_obj_align(temperature_label2_, LV_ALIGN_BOTTOM_MID, 60, -40);

    // 湿度标签
    humidity_label1_ = lv_label_create(tab_idle);
    const lv_font_t* humidity_font = font_weather_32();
    if (humidity_font != NULL) {
        lv_obj_set_style_text_font(humidity_label1_, humidity_font, LV_PART_MAIN);
    } else {
        ESP_LOGE(TAG, "Failed to get weather font for humidity");
    }
    lv_obj_align(humidity_label1_, LV_ALIGN_BOTTOM_MID, 10, -10);
    weather_code_to_utf8(901, icon_buf);
    lv_label_set_text(humidity_label1_, icon_buf);

    humidity_label2_ = lv_label_create(tab_idle);
    lv_label_set_text(humidity_label2_, "");//65%
    lv_obj_align(humidity_label2_, LV_ALIGN_BOTTOM_MID, 60, -15);

    //提醒标签
    hint_label_ = lv_label_create(tab_idle);
    lv_obj_set_style_text_color(hint_label_, lv_color_hex(0xF1BA3B), 0);
    lv_label_set_text(hint_label_, "");//提醒
    lv_obj_align(hint_label_, LV_ALIGN_BOTTOM_MID, 0, 15);
#endif
}

void XiaoziyunliaoDisplay::UpdateIdleScreen() {
    // Get current time
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    DisplayLockGuard lock(this);
    // 更新时间
    if (hour_label_) {
        char hour_str[6];
        sprintf(hour_str, "%02d ", timeinfo.tm_hour);
        lv_label_set_text(hour_label_, hour_str);
    }
    if (minute_label_) {
        char minute_str[3];
        sprintf(minute_str, "%02d", timeinfo.tm_min);
        lv_label_set_text(minute_label_, minute_str);
    }
    if (colon_label_) {
        if (timeinfo.tm_sec % 2 == 0) {
            lv_obj_set_style_text_color(colon_label_, lv_color_white(), 0);
        } else {
            lv_obj_set_style_text_color(colon_label_, lv_color_black(), 0);
        }
    }
    // 更新日期
    if (date_label_) {
        char date_str[25];
        snprintf(date_str, sizeof(date_str), Lang::Strings::DATE, timeinfo.tm_mon + 1, timeinfo.tm_mday);
        lv_label_set_text(date_label_, date_str);
    }
    // 更新星期
    if (weekday_label_ && timeinfo.tm_wday >= 0 && timeinfo.tm_wday < 7) {
        const char *weekdays[] = {Lang::Strings::WEEK7, Lang::Strings::WEEK1, Lang::Strings::WEEK2,
            Lang::Strings::WEEK3, Lang::Strings::WEEK4, Lang::Strings::WEEK5, Lang::Strings::WEEK6};
        lv_label_set_text(weekday_label_, weekdays[timeinfo.tm_wday]);
    }

#if CONFIG_USE_WEATHER
    WeatherInfo weatherinfo = WeatherForecast::GetInstance().GetWeatherInfo();
    int secinter = 1;
    if(weatherinfo.city_code > 0 && weatherinfo.update_time > 0){
        secinter = 60;
    }
    if (timeinfo.tm_sec % secinter == 0) {
        int32_t city_code = -1;

        Settings settings("display", true);
        int isUseLast = settings.GetBool("use_last", false);
        auto board = static_cast<XiaoZhiYunliaoS3*>(&Board::GetInstance());
        if (isUseLast || board->GetNetworkType() == NetworkType::ML307) {
            city_code = settings.GetInt("city_code", -1);
        }
        
        int32_t last_city_code = weatherinfo.city_code;
        // ESP_LOGI(TAG, "last_city_code: %ld", weatherinfo.city_code);
        WeatherInfo weatherinfo = WeatherForecast::GetInstance().GetWeatherForecast(
            Board::GetInstance().GetNetwork()->CreateHttp(0), city_code);

        if(weatherinfo.city_code > 0 && (weatherinfo.city_code != last_city_code)){
            ESP_LOGI(TAG, "save city_code: %ld", weatherinfo.city_code);
            settings.SetInt("city_code", weatherinfo.city_code);
        }
        if(weatherinfo.city_code > 0 && weatherinfo.update_time > 0){
            // 更新城市
            if (city_label_) {
                lv_label_set_text(city_label_, weatherinfo.city.c_str());
            }
            // 更新AQI
            if (aqi_label_) {
                const char *text = weatherinfo.quality.c_str();
                if (strcmp(text, "优") == 0 || strcmp(text, "良") == 0){
                    char aqi_str[20];
                    snprintf(aqi_str, sizeof(aqi_str), "空气%s", text);
                    lv_label_set_text(aqi_label_, aqi_str);
                }else{
                    lv_label_set_text(aqi_label_, text);
                }
                // 根据文本设置背景颜色
                lv_color_t bg_color;
                if (strcmp(text, "优") == 0) {
                    bg_color = lv_color_hex(0x18EB4A); // 绿色
                } else if (strcmp(text, "良") == 0) {
                    bg_color = lv_color_hex(0xFFF784); // 黄色
                } else if (strcmp(text, "轻度污染") == 0) {
                    bg_color = lv_color_hex(0xFF4500); // 橙色
                } else if (strcmp(text, "中度污染") == 0) {
                    bg_color = lv_color_hex(0xFF1800); // 红色
                } else if (strcmp(text, "重度污染") == 0) {
                    bg_color = lv_color_hex(0xDE71DE); // 深红色
                } else{
                    bg_color = lv_color_hex(0xDE71DE); // 紫色
                }
                lv_obj_set_style_bg_color(aqi_label_, bg_color, 0);
            }
            // 更新天气
            if (weather_label_) {
                lv_label_set_text(weather_label_, weatherinfo.weather_type.c_str());
            }
            // 更新温度
            if (temperature_label2_) {
                char temp_str[30];
                snprintf(temp_str, sizeof(temp_str), "%s~%s", weatherinfo.low_temp.c_str(), weatherinfo.high_temp.c_str());
                lv_label_set_text(temperature_label2_, temp_str);
            }
            // 更新湿度
            if (humidity_label2_) {
                lv_label_set_text(humidity_label2_, weatherinfo.humidity.c_str());
            }
            // 更新风向
            if (wind_label_) {
                char wind_str[30];
                snprintf(wind_str, sizeof(wind_str), "%s %s", weatherinfo.wind_direction.c_str(), weatherinfo.wind_power.c_str());
                lv_label_set_text(wind_label_, wind_str);
            }
            // 更新提示
            if (hint_label_) {
                lv_label_set_text(hint_label_, weatherinfo.notice.c_str());
            }
        }
    }
#endif
}

void XiaoziyunliaoDisplay::ShowStandbyScreen(bool show) {
    if (tabview_) {
        // 在切换标签页前加锁，防止异常
        lv_lock();
        if (show){
            lv_tabview_set_act(tabview_, (uint32_t) PageIndex::PAGE_IDLE, LV_ANIM_OFF);
            if (!idle_timer_created_) {
                auto updateFunc = [](lv_timer_t *t) {
                    static_cast<XiaoziyunliaoDisplay*>(lv_timer_get_user_data(t))->UpdateIdleScreen();
                };
                lv_timer_create(updateFunc, 1000, this);
                idle_timer_created_ = true;
            }
        } else {
            lv_tabview_set_act(tabview_, (uint32_t) PageIndex::PAGE_CHAT, LV_ANIM_OFF);
        }
        lv_unlock();
    }
}


void XiaoziyunliaoDisplay::SetChatMessage(const char* role, const char* content) {
    DisplayLockGuard lock(this);
    LcdDisplay::SetChatMessage(role, content);
    // lv_obj_scroll_to_view_recursive(chat_message_label_, LV_ANIM_OFF);

}

void XiaoziyunliaoDisplay::NewChatPage() {
    LvglTheme* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    
    DisplayLockGuard lock(this);
    DelChatPage();

    if (isActivationStatus()) {
        // 创建带边框的二维码容器
        qr_container = lv_obj_create(content_);
        lv_obj_remove_style_all(qr_container);
        lv_obj_set_size(qr_container, 106, 106);
        lv_obj_set_style_border_color(qr_container, lvgl_theme->text_color(), 0);
        lv_obj_set_style_border_width(qr_container, 3, 0);
        lv_obj_set_style_bg_color(qr_container, lvgl_theme->background_color(), 0);
        lv_obj_set_style_pad_all(qr_container, 0, 0);

        // 创建控制台二维码
        console_qrcode_ = lv_qrcode_create(qr_container);
        lv_qrcode_set_size(console_qrcode_, 100);
        lv_qrcode_set_dark_color(console_qrcode_, lvgl_theme->background_color());
        lv_qrcode_set_light_color(console_qrcode_, lvgl_theme->text_color());
        lv_qrcode_update(console_qrcode_, CONSOLE_URL, strlen(CONSOLE_URL));
        lv_obj_center(console_qrcode_);

        if (emoji_box_) {
            lv_obj_del(emoji_box_);
            emoji_box_ = nullptr;
        }
    } else {
        // 创建表情标签
        emoji_box_ = lv_obj_create(content_);
        lv_obj_set_size(emoji_box_, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(emoji_box_, LV_OPA_TRANSP, 0);
        lv_obj_set_style_pad_all(emoji_box_, 0, 0);
        lv_obj_set_style_border_width(emoji_box_, 0, 0);

        emoji_label_ = lv_label_create(emoji_box_);
        lv_obj_set_style_text_font(emoji_label_, lvgl_theme->large_icon_font()->font(), 0);
        lv_obj_set_style_text_color(emoji_label_, lvgl_theme->text_color(), 0);
        // lv_label_set_text(emoji_label_, FONT_AWESOME_MICROCHIP_AI);

        emoji_image_ = lv_img_create(emoji_box_);
        lv_obj_center(emoji_image_);
        lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);

        preview_image_ = lv_image_create(content_);
        lv_obj_set_size(preview_image_, width_ / 2, height_ / 2);
        lv_obj_align(preview_image_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
    }

    // 创建聊天消息标签
    chat_message_label_ = lv_label_create(content_);
    lv_label_set_text(chat_message_label_, "");
    lv_obj_set_width(chat_message_label_, width_ * 0.9); // 限制宽度为屏幕宽度的 90%
    lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_WRAP); // 设置为自动换行模式
    lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_CENTER, 0); // 设置文本居中对齐
    lv_obj_set_style_text_color(chat_message_label_, lvgl_theme->text_color(), 0);
}

void XiaoziyunliaoDisplay::SetStatus(const char* status) {
    DisplayLockGuard lock(this);
    std::string old_status = current_status_;
    current_status_ = status ? status : "";
    if ((old_status != Lang::Strings::ACTIVATION && isActivationStatus()) ||
        (old_status == Lang::Strings::ACTIVATION && !isActivationStatus())) {
        DelConfigPage();
        NewChatPage();
        lv_page_index = PageIndex::PAGE_CHAT;
    }
    SpiLcdDisplay::SetStatus(status);    
}

void XiaoziyunliaoDisplay::SwitchPage(std::optional<PageIndex> target) {
    DisplayLockGuard lock(this);
    PageIndex new_page = target.value_or(
        (lv_page_index == PageIndex::PAGE_CHAT) ? 
        PageIndex::PAGE_CONFIG : PageIndex::PAGE_CHAT
    );

    const bool is_chat_page = (new_page == PageIndex::PAGE_CHAT);
    
    if (is_chat_page) {
        DelConfigPage();
        ShowChatPage();
    } else {
        HideChatPage();
        NewConfigPage();
    }
    lv_page_index = new_page;
}
void XiaoziyunliaoDisplay::NewSmartConfigPage() {
    LvglTheme* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    DisplayLockGuard lock(this);
    DelConfigPage();
    ShowChatPage();
    if (emoji_box_) {
        lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
    }
    // if (emoji_image_) {
    //     lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    // }
    if (content_) {
        smartconfig_qrcode_ = lv_qrcode_create(content_);
        lv_qrcode_set_size(smartconfig_qrcode_, 100);
        lv_qrcode_set_dark_color(smartconfig_qrcode_, lvgl_theme->background_color());
        lv_qrcode_set_light_color(smartconfig_qrcode_, lvgl_theme->text_color());
        lv_qrcode_update(smartconfig_qrcode_, WIFI_URL, strlen(WIFI_URL));
    }
}

void XiaoziyunliaoDisplay::HideSmartConfigPage() {
    DisplayLockGuard lock(this);
    if (smartconfig_qrcode_) {
        lv_obj_add_flag(smartconfig_qrcode_, LV_OBJ_FLAG_HIDDEN);
    }
    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void XiaoziyunliaoDisplay::NewConfigPage() {
    DisplayLockGuard lock(this);
    DelConfigPage();
    auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    auto text_font = lvgl_theme->text_font()->font();
    // 创建配置页面
    config_container_ = lv_obj_create(content_);
    lv_obj_remove_style_all(config_container_);
    lv_obj_set_size(config_container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_flex_flow(config_container_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(config_container_, 10, 0);
    lv_obj_set_style_pad_top(config_container_, 23, 0);
    lv_obj_set_style_flex_main_place(config_container_, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_flex_cross_place(config_container_, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(config_container_, lvgl_theme->text_color(), 0);

    uint8_t left_width =  170;
    uint8_t right_width = 140;
#if defined (zh_tw)
    left_width =  160;
    right_width = 140;
#elif defined (ja_jp)
    left_width =  140;
    right_width = 130;
#elif defined (en_us)
    left_width =  160;
    right_width = 140;
#endif

    // 左侧文本说明区
    config_text_panel_ = lv_label_create(config_container_);
    lv_obj_set_width(config_text_panel_, LV_HOR_RES - left_width);
    std::string hint_text = Lang::Strings::HINT1;
    hint_text += "\n  ";
    hint_text += Lang::Strings::HINT2;
    hint_text += "\n  ";
    hint_text += Lang::Strings::HINT3;
    hint_text += "\n  ";
    hint_text += Lang::Strings::HINT4;
    hint_text += "\n  ";
    hint_text += Lang::Strings::HINT5;
    hint_text += "\n";
    hint_text += Lang::Strings::HINT6;
    hint_text += "\n  ";
    hint_text += Lang::Strings::HINT7;
    hint_text += "\n  ";
    hint_text += Lang::Strings::HINT8;
    lv_label_set_text(config_text_panel_, hint_text.c_str());
    // lv_obj_set_style_text_font(config_text_panel_, text_font, 0);
    lv_label_set_long_mode(config_text_panel_, LV_LABEL_LONG_WRAP);

    /* 右侧二维码区 */
    right_container = lv_obj_create(config_container_);
    lv_obj_remove_style_all(right_container);
    lv_obj_set_size(right_container, right_width, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(right_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(right_container, 5, 0);
    lv_obj_set_style_flex_main_place(right_container, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_flex_cross_place(right_container, LV_FLEX_ALIGN_CENTER, 0);

    qrcode_label_ = lv_label_create(right_container);
    std::string qrcode_text = Lang::Strings::LOGO;
    qrcode_text += Lang::Strings::VERSION3;
    qrcode_text += "\n";
    qrcode_text += Lang::Strings::HELP3;
    lv_label_set_text(qrcode_label_, qrcode_text.c_str());
    lv_obj_set_style_text_font(qrcode_label_, text_font, 0);
    lv_obj_set_style_text_line_space(qrcode_label_, 2, 0);
    lv_obj_set_style_text_align(qrcode_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(qrcode_label_, LV_PCT(100));

    // 直接创建二维码
    config_qrcode_panel_ = lv_qrcode_create(right_container);
    lv_qrcode_set_size(config_qrcode_panel_, 120);
    lv_qrcode_set_dark_color(config_qrcode_panel_, lvgl_theme->background_color());
    lv_qrcode_set_light_color(config_qrcode_panel_, lvgl_theme->text_color());
    lv_obj_center(config_qrcode_panel_);
    lv_qrcode_update(config_qrcode_panel_, CONSOLE_URL, strlen(CONSOLE_URL));
}

void XiaoziyunliaoDisplay::DelConfigPage() {
    DisplayLockGuard lock(this);
    if (config_text_panel_) {
        lv_obj_del(config_text_panel_);
        config_text_panel_ = nullptr;
    }
    if (config_qrcode_panel_) {
        lv_obj_del(config_qrcode_panel_);
        config_qrcode_panel_ = nullptr;
    }
    if (qrcode_label_) {
        lv_obj_del(qrcode_label_);
        qrcode_label_ = nullptr;
    }
    if (right_container) {
        lv_obj_del(right_container);
        right_container = nullptr;
    }
    if (config_container_) {
        lv_obj_del(config_container_);
        config_container_ = nullptr;
    }
}


void XiaoziyunliaoDisplay::HideChatPage() {
    DisplayLockGuard lock(this);
    if (qr_container) {
        lv_obj_add_flag(qr_container, LV_OBJ_FLAG_HIDDEN);
    }
    if (console_qrcode_) {
        lv_obj_add_flag(console_qrcode_, LV_OBJ_FLAG_HIDDEN);
    }
    if (emoji_box_) {
        lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
    }
    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void XiaoziyunliaoDisplay::ShowChatPage() {
    DisplayLockGuard lock(this);
    if (isActivationStatus()) {
        if (qr_container) {
            lv_obj_remove_flag(qr_container, LV_OBJ_FLAG_HIDDEN);
        }
        if (console_qrcode_) {
            lv_obj_remove_flag(console_qrcode_, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        if (emoji_box_) {
            lv_obj_remove_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (chat_message_label_) {
        lv_obj_remove_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void XiaoziyunliaoDisplay::DelChatPage() {
    DisplayLockGuard lock(this);
    if (qr_container) {
        lv_obj_del(qr_container);
        qr_container = nullptr;
    }
    if (console_qrcode_) {
        lv_obj_del(console_qrcode_);
        console_qrcode_ = nullptr;
    }
    if (emoji_box_) {
        lv_obj_del(emoji_box_);
        emoji_box_ = nullptr;
    }
    if (chat_message_label_) {
        lv_obj_del(chat_message_label_);
        chat_message_label_ = nullptr;
    }
}

bool XiaoziyunliaoDisplay::isActivationStatus() const {
    return current_status_ == Lang::Strings::ACTIVATION;
}

bool XiaoziyunliaoDisplay::isWifiConfigStatus() const {
    return current_status_ == Lang::Strings::WIFI_CONFIG_MODE;
}

XiaoziyunliaoDisplay::~XiaoziyunliaoDisplay() {
    DisplayLockGuard lock(this);
    current_status_.clear();
    
    if (config_container_) { lv_obj_del(config_container_); config_container_ = nullptr; }
    if (container_) { lv_obj_del(container_); container_ = nullptr; }
    if (status_bar_) { lv_obj_del(status_bar_); status_bar_ = nullptr; }
    if (logo_label_) { lv_obj_del(logo_label_); logo_label_ = nullptr; }
    if (notification_label_) { lv_obj_del(notification_label_); notification_label_ = nullptr; }
    if (status_label_) { lv_obj_del(status_label_); status_label_ = nullptr; }
    if (mute_label_) { lv_obj_del(mute_label_); mute_label_ = nullptr; }
    if (network_label_) { lv_obj_del(network_label_); network_label_ = nullptr; }
    if (battery_label_) { lv_obj_del(battery_label_); battery_label_ = nullptr; }
    if (content_) { lv_obj_del(content_); content_ = nullptr; }
    if (smartconfig_qrcode_) { lv_obj_del(smartconfig_qrcode_); smartconfig_qrcode_ = nullptr; }
}

void XiaoziyunliaoDisplay::SetEmotion(const char* emotion) {
    DisplayLockGuard lock(this);
    LcdDisplay::SetEmotion(emotion);
}

void XiaoziyunliaoDisplay::SetTheme(Theme* theme) {
    DisplayLockGuard lock(this);
    if (logo_label_) {
        lv_label_set_text(logo_label_, Lang::Strings::LOGO);
    }
    ESP_LOGI(TAG, "Set theme to %s", theme->name().c_str());
    LvglTheme* lvgl_theme = static_cast<LvglTheme*>(theme);
    LcdDisplay::SetTheme(lvgl_theme);
    lv_obj_set_style_text_font(tab_main, lvgl_theme->text_font()->font(), 0);
    lv_obj_set_style_text_color(tab_main, lvgl_theme->text_color(), 0);
    lv_obj_set_style_bg_color(tab_main, lvgl_theme->background_color(), 0);
    
    lv_obj_set_style_text_font(tab_idle, lvgl_theme->text_font()->font(), 0);
    lv_obj_set_style_text_color(tab_idle, lv_color_white(), 0);
    lv_obj_set_style_bg_color(tab_idle, lv_color_black(), 0);
}

void XiaoziyunliaoDisplay::showUI() {
    std::string helpMessage = Lang::Strings::HELP1;
    helpMessage += "\n"; 
    helpMessage += Lang::Strings::HELP2;
    DisplayLockGuard lock(this);
    SpiLcdDisplay::SetChatMessage("system", helpMessage.c_str());
}
