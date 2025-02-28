#include "xiaoziyunliao_display.h"
#include "xiaozhiyunliao_c3.h"
#include "assets/lang_config.h"
#include <font_awesome_symbols.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/ledc.h>
#include <vector>
#include <esp_lvgl_port.h>
#include "board.h"
#include <string.h> 

#define TAG "XiaoziyunliaoDisplay"

LV_FONT_DECLARE(font_awesome_30_4);


XiaoziyunliaoDisplay::XiaoziyunliaoDisplay(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_handle_t panel,
    gpio_num_t backlight_pin,
    bool backlight_output_invert,
    int width, int height,
    int offset_x, int offset_y,
    bool mirror_x, bool mirror_y,
    bool swap_xy,
    DisplayFonts fonts)
    : SpiLcdDisplay(panel_io, panel,
                backlight_pin,
                backlight_output_invert,
                width, height,
                offset_x, offset_y,
                mirror_x, mirror_y,
                swap_xy,
                fonts){
        SetupUI();
        SetBacklight(60);
        SetLogo(Lang::Strings::LOGO);
}


void XiaoziyunliaoDisplay::SetupUI() {
    DisplayLockGuard lock(this);

    auto screen = lv_screen_active();
    lv_obj_set_style_text_font(screen, fonts_.text_font, 0);
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    

    /* Container */
    container_ = lv_obj_create(screen);
    lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(container_, 0, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_style_pad_row(container_, 0, 0);

    /* Status bar */
    status_bar_ = lv_obj_create(container_);
    lv_obj_set_size(status_bar_, LV_HOR_RES, fonts_.text_font->line_height);
    lv_obj_set_style_radius(status_bar_, 0, 0);
    lv_obj_set_style_text_color(status_bar_, lv_color_make(0xAf, 0xAf, 0xAf), 0);
    lv_obj_set_style_bg_color(status_bar_, lv_color_black(), 0);
    
    /* Status bar */
    lv_obj_set_flex_flow(status_bar_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(status_bar_, 0, 0);
    lv_obj_set_style_border_width(status_bar_, 0, 0);
    lv_obj_set_style_pad_column(status_bar_, 0, 0);
    lv_obj_set_style_pad_left(status_bar_, 2, 0);
    lv_obj_set_style_pad_right(status_bar_, 2, 0);

    logo_label_ = lv_label_create(status_bar_);
    lv_label_set_text(logo_label_, "");
    lv_obj_set_style_text_font(logo_label_, fonts_.text_font, 0);

    notification_label_ = lv_label_create(status_bar_);
    lv_obj_set_flex_grow(notification_label_, 1);
    lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(notification_label_, "");
    lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

    status_label_ = lv_label_create(status_bar_);
    lv_obj_set_flex_grow(status_label_, 1);
    lv_label_set_long_mode(status_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);
    lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);

    mute_label_ = lv_label_create(status_bar_);
    lv_label_set_text(mute_label_, "");
    lv_obj_set_style_text_font(mute_label_, fonts_.icon_font, 0);

    network_label_ = lv_label_create(status_bar_);
    lv_label_set_text(network_label_, "");
    lv_obj_set_style_text_font(network_label_, fonts_.icon_font, 0);
    lv_obj_set_style_pad_left(network_label_, 3, 0);

    battery_label_ = lv_label_create(status_bar_);
    lv_label_set_text(battery_label_, "");
    lv_obj_set_style_text_font(battery_label_, fonts_.icon_font, 0);
    lv_obj_set_style_pad_left(battery_label_, 3, 0);

    /* Content */
    content_ = lv_obj_create(container_);
    lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_radius(content_, 0, 0);
    lv_obj_set_width(content_, LV_HOR_RES);
    lv_obj_set_flex_grow(content_, 1);
    
    lv_obj_set_flex_flow(content_, LV_FLEX_FLOW_COLUMN); // 垂直布局（从上到下）
    lv_obj_set_flex_align(content_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY); // 子对象居中对齐，等距分布
    lv_obj_set_style_border_width(content_, 0, 0);
    lv_obj_set_style_bg_color(content_, lv_color_black(), 0);
    lv_obj_set_style_text_color(content_, lv_color_white(), 0);

    NewChatPage();
}

void XiaoziyunliaoDisplay::SetStatus(const char* status) {
    DisplayLockGuard lock(this);
    std::string old_status = current_status_;
    current_status_ = status ? status : "";
    if ((old_status != Lang::Strings::ACTIVATION && isActivationStatus()) ||
        (old_status == Lang::Strings::ACTIVATION && !isActivationStatus())) {
        NewChatPage();
    }
    LcdDisplay::SetStatus(status);    
}

void XiaoziyunliaoDisplay::SwitchPage(std::optional<PageIndex> target) {
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
    DisplayLockGuard lock(this);
    if (emotion_label_) {
        lv_obj_add_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
    }

    if (content_) {
        smartconfig_qrcode_ = lv_qrcode_create(content_);
        lv_qrcode_set_size(smartconfig_qrcode_, 100);
        lv_qrcode_set_dark_color(smartconfig_qrcode_, lv_color_white());
        lv_qrcode_set_light_color(smartconfig_qrcode_, lv_color_black());
        lv_qrcode_update(smartconfig_qrcode_, "https://iot.espressif.cn/configWXDeviceWiFi.html", strlen("https://iot.espressif.cn/configWXDeviceWiFi.html"));
    }
}

void XiaoziyunliaoDisplay::HideSmartConfigPage() {
    if (smartconfig_qrcode_) {
        lv_obj_add_flag(smartconfig_qrcode_, LV_OBJ_FLAG_HIDDEN);
    }
    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void XiaoziyunliaoDisplay::SetLogo(const char* logo) {
    DisplayLockGuard lock(this);
    if (logo_label_) {
        lv_label_set_text(logo_label_, logo);
    }
}

void XiaoziyunliaoDisplay::NewConfigPage() {
    DisplayLockGuard lock(this);
    DelConfigPage();
    // 创建配置页面
    config_container_ = lv_obj_create(content_);
    lv_obj_remove_style_all(config_container_);
    lv_obj_set_size(config_container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_flex_flow(config_container_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(config_container_, 10, 0);
    lv_obj_set_style_pad_top(config_container_, 23, 0);
    lv_obj_set_style_flex_main_place(config_container_, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_flex_cross_place(config_container_, LV_FLEX_ALIGN_CENTER, 0);

    // 左侧文本说明区
    config_text_panel_ = lv_label_create(config_container_);
    lv_obj_set_width(config_text_panel_, LV_HOR_RES - 150 - 20);
    // std::string qrcode_text = Lang::Strings::HELP3;

    lv_label_set_text(config_text_panel_,
                    "AI语音指令:\n"
                    "  单击/长按通话\n"
                    "  关机/重启\n"
                    "  亮度音量电量\n"
                    "  重新配网\n"
                    "按键功能:\n"
                    "  双击切换页面\n"
                    "  戳小孔重启"
                    );
    lv_obj_set_style_text_font(config_text_panel_, fonts_.text_font, 0);
    lv_label_set_long_mode(config_text_panel_, LV_LABEL_LONG_WRAP);

    /* 右侧二维码区 */
    lv_obj_t* right_container = lv_obj_create(config_container_);
    lv_obj_remove_style_all(right_container);
    lv_obj_set_size(right_container, 140, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(right_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(right_container, 5, 0);
    lv_obj_set_style_flex_main_place(right_container, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_flex_cross_place(right_container, LV_FLEX_ALIGN_CENTER, 0);

    qrcode_label_ = lv_label_create(right_container);
    std::string qrcode_text = Lang::Strings::HELP3;
    qrcode_text += "\n";
    qrcode_text += Lang::Strings::HELP4;
    lv_label_set_text(qrcode_label_, qrcode_text.c_str());
    lv_obj_set_style_text_font(qrcode_label_, fonts_.text_font, 0);
    lv_obj_set_style_text_line_space(qrcode_label_, 2, 0);
    lv_obj_set_style_text_align(qrcode_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(qrcode_label_, LV_PCT(100));

    // 创建带边框的容器
    lv_obj_t* border_container = lv_obj_create(right_container);
    lv_obj_remove_style_all(border_container);
    lv_obj_set_size(border_container, 126, 126);
    lv_obj_set_style_border_color(border_container, lv_color_white(), 0);
    lv_obj_set_style_border_width(border_container, 3, 0);
    lv_obj_set_style_bg_color(border_container, lv_color_black(), 0);
    lv_obj_set_style_pad_all(border_container, 0, 0);

    // 在边框容器中创建二维码
    config_qrcode_panel_ = lv_qrcode_create(border_container);
    lv_qrcode_set_size(config_qrcode_panel_, 120);
    lv_qrcode_set_dark_color(config_qrcode_panel_, lv_color_black());
    lv_qrcode_set_light_color(config_qrcode_panel_, lv_color_white());
    lv_obj_center(config_qrcode_panel_);
    lv_qrcode_update(config_qrcode_panel_, "https://xiaozhi.me/", strlen("https://xiaozhi.me/"));
}

void XiaoziyunliaoDisplay::DelConfigPage() {
    if (config_container_) {
        lv_obj_del(config_container_);
        config_container_ = nullptr;
    }
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
}

void XiaoziyunliaoDisplay::NewChatPage() {
    DisplayLockGuard lock(this);
    DelChatPage();

    if (isActivationStatus()) {
        // 创建带边框的二维码容器
        qr_container = lv_obj_create(content_);
        lv_obj_remove_style_all(qr_container);
        lv_obj_set_size(qr_container, 126, 126);
        lv_obj_set_style_border_color(qr_container, lv_color_white(), 0);
        lv_obj_set_style_border_width(qr_container, 3, 0);
        lv_obj_set_style_bg_color(qr_container, lv_color_black(), 0);
        lv_obj_set_style_pad_all(qr_container, 0, 0);

        // 创建控制台二维码
        console_qrcode_ = lv_qrcode_create(qr_container);
        lv_qrcode_set_size(console_qrcode_, 120);
        lv_qrcode_set_dark_color(console_qrcode_, lv_color_black());
        lv_qrcode_set_light_color(console_qrcode_, lv_color_white());
        lv_qrcode_update(console_qrcode_, "https://xiaozhi.me/console/agents", strlen("https://xiaozhi.me/console/agents"));
        lv_obj_center(console_qrcode_);

        if (emotion_label_) {
            lv_obj_del(emotion_label_);
            emotion_label_ = nullptr;
        }
    } else {
        // 创建表情标签
        emotion_label_ = lv_label_create(content_);
        lv_obj_set_style_text_font(emotion_label_, &font_awesome_30_4, 0);
        lv_label_set_text(emotion_label_, FONT_AWESOME_AI_CHIP);
        
        if (console_qrcode_) {
            lv_obj_del(console_qrcode_);
            console_qrcode_ = nullptr;
        }
    }

    // 创建聊天消息标签
    chat_message_label_ = lv_label_create(content_);
    lv_label_set_text(chat_message_label_, "");
    lv_obj_set_width(chat_message_label_, LV_HOR_RES * 0.9);
    lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_CENTER, 0);
}

void XiaoziyunliaoDisplay::HideChatPage() {
    if (qr_container) {
        lv_obj_add_flag(qr_container, LV_OBJ_FLAG_HIDDEN);
    }
    if (console_qrcode_) {
        lv_obj_add_flag(console_qrcode_, LV_OBJ_FLAG_HIDDEN);
    }
    if (emotion_label_) {
        lv_obj_add_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
    }
    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void XiaoziyunliaoDisplay::ShowChatPage() {
    if (isActivationStatus()) {
        if (qr_container) {
            lv_obj_clear_flag(qr_container, LV_OBJ_FLAG_HIDDEN);
        }
        if (console_qrcode_) {
            lv_obj_clear_flag(console_qrcode_, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        if (emotion_label_) {
            lv_obj_clear_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (chat_message_label_) {
        lv_obj_clear_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void XiaoziyunliaoDisplay::DelChatPage() {
    if (qr_container) {
        lv_obj_del(qr_container);
        qr_container = nullptr;
    }
    if (console_qrcode_) {
        lv_obj_del(console_qrcode_);
        console_qrcode_ = nullptr;
    }
    if (emotion_label_) {
        lv_obj_del(emotion_label_);
        emotion_label_ = nullptr;
    }
    if (chat_message_label_) {
        lv_obj_del(chat_message_label_);
        chat_message_label_ = nullptr;
    }
}

bool XiaoziyunliaoDisplay::isActivationStatus() const {
    return current_status_ == Lang::Strings::ACTIVATION;
}

XiaoziyunliaoDisplay::~XiaoziyunliaoDisplay() {
    DisplayLockGuard lock(this);
    current_status_.clear();
    DelConfigPage();
    DelChatPage(); 
    
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

