#include "mogai_cuican_display.h"
#include "mogai_cuican_esp32s3.h"
#include "assets/lang_config.h"
#include <font_awesome_symbols.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/ledc.h>
#include <vector>
#include <esp_lvgl_port.h>
#include "board.h"
#include <string.h>

#define TAG "CuicanDisplay"

LV_FONT_DECLARE(font_awesome_20_4);

constexpr char CONSOLE_URL[] = "https://xiaozhi.me/console";
constexpr char WIFI_URL[] = "https://iot.espressif.cn/configWXDeviceWiFi.html";

MogaiCuicanDisplay::MogaiCuicanDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                           int width, int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y, bool swap_xy,
                           DisplayFonts fonts)
    : SpiLcdDisplay(panel_io, panel,
                width, height,
                offset_x, offset_y,
                mirror_x, mirror_y,
                swap_xy,
                fonts){
        SetupUI();
}


void MogaiCuicanDisplay::SetupUI() {
    DisplayLockGuard lock(this);
    SpiLcdDisplay::SetupUI();
    // NewChatPage();
}

    
// void MogaiCuicanDisplay::SetChatMessage(const char* role, const char* content) {
//     DisplayLockGuard lock(this);

//     SpiLcdDisplay::SetChatMessage(role, content);
 
// }

void MogaiCuicanDisplay::NewChatPage() {
    DisplayLockGuard lock(this);
    DelChatPage();

    if (isActivationStatus()) {
        // 创建带边框的二维码容器
        qr_container = lv_obj_create(content_);
        lv_obj_remove_style_all(qr_container);
        lv_obj_set_size(qr_container, 82, 82);
        lv_obj_set_style_border_color(qr_container, lv_color_white(), 0);
        lv_obj_set_style_border_width(qr_container, 1, 0);
        lv_obj_set_style_bg_color(qr_container, lv_color_black(), 0);
        lv_obj_set_style_pad_all(qr_container, 0, 0);

        // 创建控制台二维码
        console_qrcode_ = lv_qrcode_create(qr_container);
        lv_qrcode_set_size(console_qrcode_, 80);
        lv_qrcode_set_dark_color(console_qrcode_, lv_color_black());
        lv_qrcode_set_light_color(console_qrcode_, lv_color_white());
        lv_qrcode_update(console_qrcode_, CONSOLE_URL, strlen(CONSOLE_URL));
        lv_obj_center(console_qrcode_);

        if (emotion_label_) {
            lv_obj_del(emotion_label_);
            emotion_label_ = nullptr;
        }
    } else {
        // 创建表情标签
        emotion_label_ = lv_label_create(content_);
        lv_obj_set_style_text_font(emotion_label_, &font_awesome_20_4, 0);
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
    lv_obj_set_style_text_color(chat_message_label_, lv_color_white(), 0);
}
void MogaiCuicanDisplay::SetStatus(const char* status) {
    DisplayLockGuard lock(this);
    std::string old_status = current_status_;
    current_status_ = status ? status : "";
    if ((old_status != Lang::Strings::ACTIVATION && isActivationStatus()) ||
        (old_status == Lang::Strings::ACTIVATION && !isActivationStatus())) {
        DelConfigPage();
        NewChatPage();
        lv_page_index = PageIndex::PAGE_CHAT;
    }
    LcdDisplay::SetStatus(status);    
}

void MogaiCuicanDisplay::SwitchPage(std::optional<PageIndex> target) {
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



void MogaiCuicanDisplay::NewSmartConfigPage() {
    DisplayLockGuard lock(this);
    // DelConfigPage();0
    // ShowChatPage();
    if (emotion_label_) {
        lv_obj_add_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
    }
    if (content_) {
        smartconfig_qrcode_ = lv_qrcode_create(content_);
        lv_qrcode_set_size(smartconfig_qrcode_, 80);
        lv_qrcode_set_dark_color(smartconfig_qrcode_, lv_color_white());
        lv_qrcode_set_light_color(smartconfig_qrcode_, lv_color_black());
        lv_qrcode_update(smartconfig_qrcode_, WIFI_URL, strlen(WIFI_URL));
    }
}

void MogaiCuicanDisplay::HideSmartConfigPage() {
    DisplayLockGuard lock(this);
    if (smartconfig_qrcode_) {
        lv_obj_add_flag(smartconfig_qrcode_, LV_OBJ_FLAG_HIDDEN);
    }
    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}


#if defined(ja_jp)
    LV_FONT_DECLARE(font_noto_14_1_ja_jp);
    #define FONT2 &font_noto_14_1_ja_jp
#elif defined(en_us)
    LV_FONT_DECLARE(font_puhui_14_1);
    #define FONT2 &font_puhui_14_1
#else
    #define FONT2 fonts_.text_font
#endif

void MogaiCuicanDisplay::NewConfigPage() {
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


    /* 右侧二维码区 */
    right_container = lv_obj_create(config_container_);
    lv_obj_remove_style_all(right_container);
    lv_obj_set_size(right_container, 180, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(right_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(right_container, 5, 0);
    lv_obj_set_style_flex_main_place(right_container, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_flex_cross_place(right_container, LV_FLEX_ALIGN_CENTER, 0);

    qrcode_label_ = lv_label_create(right_container);
    std::string qrcode_text = "魔改璀璨·AI吊坠\n";
    qrcode_text += Lang::Strings::HELP3;
    lv_label_set_text(qrcode_label_, qrcode_text.c_str());
    lv_obj_set_style_text_font(qrcode_label_, FONT2, 0);
    lv_obj_set_style_text_line_space(qrcode_label_, 2, 0);
    lv_obj_set_style_text_align(qrcode_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(qrcode_label_, LV_PCT(100));
    lv_obj_set_style_text_color(qrcode_label_, lv_color_white(), 0);

    // 直接创建二维码
    config_qrcode_panel_ = lv_qrcode_create(right_container);
    lv_qrcode_set_size(config_qrcode_panel_, 120);
    lv_qrcode_set_dark_color(config_qrcode_panel_, lv_color_white());
    lv_qrcode_set_light_color(config_qrcode_panel_, lv_color_black());
    lv_obj_center(config_qrcode_panel_);
    lv_qrcode_update(config_qrcode_panel_, CONSOLE_URL, strlen(CONSOLE_URL));
}

void MogaiCuicanDisplay::DelConfigPage() {
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


void MogaiCuicanDisplay::HideChatPage() {
    DisplayLockGuard lock(this);
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

void MogaiCuicanDisplay::ShowChatPage() {
    DisplayLockGuard lock(this);
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

void MogaiCuicanDisplay::DelChatPage() {
    DisplayLockGuard lock(this);
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

bool MogaiCuicanDisplay::isActivationStatus() const {
    return current_status_ == Lang::Strings::ACTIVATION;
}

bool MogaiCuicanDisplay::isWifiConfigStatus() const {
    return current_status_ == Lang::Strings::WIFI_CONFIG_MODE;
}


MogaiCuicanDisplay::~MogaiCuicanDisplay() {
    DisplayLockGuard lock(this);
    current_status_.clear();
    
    if (config_container_) { lv_obj_del(config_container_); config_container_ = nullptr; }
    if (container_) { lv_obj_del(container_); container_ = nullptr; }
    if (status_bar_) { lv_obj_del(status_bar_); status_bar_ = nullptr; }
    if (notification_label_) { lv_obj_del(notification_label_); notification_label_ = nullptr; }
    if (status_label_) { lv_obj_del(status_label_); status_label_ = nullptr; }
    if (mute_label_) { lv_obj_del(mute_label_); mute_label_ = nullptr; }
    if (network_label_) { lv_obj_del(network_label_); network_label_ = nullptr; }
    if (battery_label_) { lv_obj_del(battery_label_); battery_label_ = nullptr; }
    if (content_) { lv_obj_del(content_); content_ = nullptr; }
    if (smartconfig_qrcode_) { lv_obj_del(smartconfig_qrcode_); smartconfig_qrcode_ = nullptr; }
}

// void MogaiCuicanDisplay::SetEmotion(const char* emotion) {
//     DisplayLockGuard lock(this);
//     if()
//     LcdDisplay::SetEmotion(emotion);
// }
