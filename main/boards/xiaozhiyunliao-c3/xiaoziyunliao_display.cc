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
#include "gb2big.h"

#define TAG "YunliaoDisplay"

LV_FONT_DECLARE(font_awesome_30_4);

constexpr char CONSOLE_URL[] = "https://xiaozhi.me/console";
constexpr char WIFI_URL[] = "https://iot.espressif.cn/configWXDeviceWiFi.html";

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
                width, height,
                offset_x, offset_y,
                mirror_x, mirror_y,
                swap_xy,
                fonts){
        SetupUI();
        SetLogo(Lang::Strings::LOGO);
}

#if CONFIG_USE_WECHAT_MESSAGE_STYLE
// Color definitions for dark theme
#define DARK_BACKGROUND_COLOR       lv_color_black()      // Dark background
#define DARK_TEXT_COLOR             lv_color_black()           // White text
#define DARK_CHAT_BACKGROUND_COLOR  lv_color_black()     // Slightly lighter than background
#define DARK_USER_BUBBLE_COLOR      lv_color_hex(0x95EC69)     // Dark green
#define DARK_ASSISTANT_BUBBLE_COLOR lv_color_white()     // Dark gray
#define DARK_SYSTEM_BUBBLE_COLOR    lv_color_hex(0xE0E0E0)      // Medium gray
#define DARK_SYSTEM_TEXT_COLOR      lv_color_hex(0x666666)     // Light gray text
#define DARK_BORDER_COLOR           lv_color_hex(0x111111)     // Dark gray border
#define DARK_LOW_BATTERY_COLOR      lv_color_hex(0xFF0000)     // Red for dark mode

// Color definitions for light theme
#define LIGHT_BACKGROUND_COLOR       lv_color_white()           // White background
#define LIGHT_TEXT_COLOR             lv_color_black()           // Black text
#define LIGHT_CHAT_BACKGROUND_COLOR  lv_color_hex(0xE0E0E0)     // Light gray background
#define LIGHT_USER_BUBBLE_COLOR      lv_color_hex(0x95EC69)     // WeChat green
#define LIGHT_ASSISTANT_BUBBLE_COLOR lv_color_white()           // White
#define LIGHT_SYSTEM_BUBBLE_COLOR    lv_color_hex(0xE0E0E0)     // Light gray
#define LIGHT_SYSTEM_TEXT_COLOR      lv_color_hex(0x666666)     // Dark gray text
#define LIGHT_BORDER_COLOR           lv_color_hex(0xE0E0E0)     // Light gray border
#define LIGHT_LOW_BATTERY_COLOR      lv_color_black()           // Black for light mode

// Define dark theme colors
static const ThemeColors DARK_THEME = {
    .background = DARK_BACKGROUND_COLOR,
    .text = DARK_TEXT_COLOR,
    .chat_background = DARK_CHAT_BACKGROUND_COLOR,
    .user_bubble = DARK_USER_BUBBLE_COLOR,
    .assistant_bubble = DARK_ASSISTANT_BUBBLE_COLOR,
    .system_bubble = DARK_SYSTEM_BUBBLE_COLOR,
    .system_text = DARK_SYSTEM_TEXT_COLOR,
    .border = DARK_BORDER_COLOR,
    .low_battery = DARK_LOW_BATTERY_COLOR
};

// Define light theme colors
static const ThemeColors LIGHT_THEME = {
    .background = LIGHT_BACKGROUND_COLOR,
    .text = LIGHT_TEXT_COLOR,
    .chat_background = LIGHT_CHAT_BACKGROUND_COLOR,
    .user_bubble = LIGHT_USER_BUBBLE_COLOR,
    .assistant_bubble = LIGHT_ASSISTANT_BUBBLE_COLOR,
    .system_bubble = LIGHT_SYSTEM_BUBBLE_COLOR,
    .system_text = LIGHT_SYSTEM_TEXT_COLOR,
    .border = LIGHT_BORDER_COLOR,
    .low_battery = LIGHT_LOW_BATTERY_COLOR
};
static ThemeColors current_theme = DARK_THEME;
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
    lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);

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
    lv_obj_add_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);

    NewChatPage();
}
void XiaoziyunliaoDisplay::SetStatus(const char* status) {
    DisplayLockGuard lock(this);
    std::string old_status = current_status_;
    current_status_ = status ? status : "";
    if ((old_status != Lang::Strings::ACTIVATION && isActivationStatus()) ||
        (old_status == Lang::Strings::ACTIVATION && !isActivationStatus()) ||
        isWifiConfigStatus()) {
        DelConfigPage();
        NewChatPage();
        lv_page_index = PageIndex::PAGE_CHAT;
    }
    LcdDisplay::SetStatus(status);    
}
void XiaoziyunliaoDisplay::SwitchPage(std::optional<PageIndex> target) {
    DisplayLockGuard lock(this);
    PageIndex new_page = target.value_or(
        (lv_page_index == PageIndex::PAGE_CHAT) ? 
        PageIndex::PAGE_CONFIG : PageIndex::PAGE_CHAT
    );

    const bool is_chat_page = (new_page == PageIndex::PAGE_CHAT);
    lv_page_index = new_page;
    
    if (is_chat_page) {
        DelConfigPage();
        NewChatPage();
    } else {
        DelChatPage();
        NewChatPage();//初始化content_
        NewConfigPage();
    }
}
void XiaoziyunliaoDisplay::NewChatPage() {
    DisplayLockGuard lock(this);
    DelChatPage();
    if(content_) {
        lv_obj_del(content_);
        content_ = nullptr;
    }
    if (isActivationStatus() || isWifiConfigStatus() 
        || lv_page_index == PageIndex::PAGE_CONFIG) {
        if (content_) {
            lv_obj_del(content_);
            content_ = nullptr;
        }
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
            lv_qrcode_update(console_qrcode_, CONSOLE_URL, strlen(CONSOLE_URL));
            lv_obj_center(console_qrcode_);
        }
        if (isActivationStatus() || isWifiConfigStatus()) {
            // 创建聊天消息标签
            chat_message_label_ = lv_label_create(content_);
            lv_label_set_text(chat_message_label_, "");
            lv_obj_set_width(chat_message_label_, LV_HOR_RES * 0.9);
            lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_WRAP);
            lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_CENTER, 0);
        }
    } else {
        if (console_qrcode_) {
            lv_obj_del(console_qrcode_);
            console_qrcode_ = nullptr;
        }
        if (chat_message_label_) {
            lv_obj_del(chat_message_label_);
            chat_message_label_ = nullptr;
        }
        if (content_) {
            lv_obj_del(content_);
            content_ = nullptr;
        }
        /* Content - Chat area */
        content_ = lv_obj_create(container_);
        lv_obj_set_style_radius(content_, 0, 0);
        lv_obj_set_width(content_, LV_HOR_RES);
        lv_obj_set_flex_grow(content_, 1);
        lv_obj_set_style_pad_all(content_, 5, 0);
        lv_obj_set_style_bg_color(content_, current_theme.chat_background, 0);
        lv_obj_set_style_border_color(content_, current_theme.border, 0);

        // Enable scrolling for chat content
        lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scroll_dir(content_, LV_DIR_VER);
        
        // Create a flex container for chat messages
        lv_obj_set_flex_flow(content_, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(content_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        lv_obj_set_style_pad_row(content_, 10, 0); // Space between messages

        // We'll create chat messages dynamically in SetChatMessage
        chat_message_label_ = nullptr;
    }

}
#define  MAX_MESSAGES 5
void XiaoziyunliaoDisplay::SetChatMessage(const char* role, const char* content) {
    DisplayLockGuard lock(this);
    if (content_ == nullptr) {
        return;
    }
    
    //避免出现空的消息框
    if(strlen(content) == 0) return;
    
    const char* display_content = content;

    if (isActivationStatus() || isWifiConfigStatus()) {
        if (chat_message_label_ == nullptr) {
            return;
        }
        lv_label_set_text(chat_message_label_, display_content);
    }else{
        // 检查消息数量是否超过限制
        uint32_t child_count = lv_obj_get_child_cnt(content_);
        if (child_count >= MAX_MESSAGES) {
            // 删除最早的消息（第一个子对象）
            lv_obj_t* first_child = lv_obj_get_child(content_, 0);
            lv_obj_t* last_child = lv_obj_get_child(content_, child_count - 1);
            if (first_child != nullptr) {
                lv_obj_del(first_child);
            }
            // Scroll to the last message immediately
            if (last_child != nullptr) {
                lv_obj_scroll_to_view_recursive(last_child, LV_ANIM_OFF);
            }
        }
        // Create a message bubble
        lv_obj_t* msg_bubble = lv_obj_create(content_);
        lv_obj_set_style_radius(msg_bubble, 8, 0);
        lv_obj_set_scrollbar_mode(msg_bubble, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_style_border_width(msg_bubble, 1, 0);
        lv_obj_set_style_border_color(msg_bubble, current_theme.border, 0);
        lv_obj_set_style_pad_all(msg_bubble, 8, 0);

        // Create the message text
        lv_obj_t* msg_text = lv_label_create(msg_bubble);
        lv_label_set_text(msg_text, display_content);
        
        // 计算文本实际宽度
        lv_coord_t text_width = lv_txt_get_width(display_content, strlen(display_content), fonts_.text_font, 0);

        // 计算气泡宽度
        lv_coord_t max_width = LV_HOR_RES * 85 / 100 - 16;  // 屏幕宽度的85%
        lv_coord_t min_width = 20;  
        lv_coord_t bubble_width;
        
        // 确保文本宽度不小于最小宽度
        if (text_width < min_width) {
            text_width = min_width;
        }

        // 如果文本宽度小于最大宽度，使用文本宽度
        if (text_width < max_width) {
            bubble_width = text_width; 
        } else {
            bubble_width = max_width;
        }
        
        // 设置消息文本的宽度
        lv_obj_set_width(msg_text, bubble_width);  // 减去padding
        lv_label_set_long_mode(msg_text, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(msg_text, fonts_.text_font, 0);

        // 设置气泡宽度
        lv_obj_set_width(msg_bubble, bubble_width);
        lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);

        // Set alignment and style based on message role
        if (strcmp(role, "user") == 0) {
            // User messages are right-aligned with green background
            lv_obj_set_style_bg_color(msg_bubble, current_theme.user_bubble, 0);
            // Set text color for contrast
            lv_obj_set_style_text_color(msg_text, current_theme.text, 0);
            
            // 设置自定义属性标记气泡类型
            lv_obj_set_user_data(msg_bubble, (void*)"user");
            
            // Set appropriate width for content
            lv_obj_set_width(msg_bubble, LV_SIZE_CONTENT);
            lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);
            
            // Add some margin 
            lv_obj_set_style_margin_right(msg_bubble, 10, 0);
            
            // Don't grow
            lv_obj_set_style_flex_grow(msg_bubble, 0, 0);
        } else if (strcmp(role, "assistant") == 0) {
            // Assistant messages are left-aligned with white background
            lv_obj_set_style_bg_color(msg_bubble, current_theme.assistant_bubble, 0);
            // Set text color for contrast
            lv_obj_set_style_text_color(msg_text, current_theme.text, 0);
            
            // 设置自定义属性标记气泡类型
            lv_obj_set_user_data(msg_bubble, (void*)"assistant");
            
            // Set appropriate width for content
            lv_obj_set_width(msg_bubble, LV_SIZE_CONTENT);
            lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);
            
            // Add some margin
            lv_obj_set_style_margin_left(msg_bubble, -4, 0);
            
            // Don't grow
            lv_obj_set_style_flex_grow(msg_bubble, 0, 0);
        } else if (strcmp(role, "system") == 0) {
            // System messages are center-aligned with light gray background
            lv_obj_set_style_bg_color(msg_bubble, current_theme.system_bubble, 0);
            // Set text color for contrast
            lv_obj_set_style_text_color(msg_text, current_theme.system_text, 0);
            
            // 设置自定义属性标记气泡类型
            lv_obj_set_user_data(msg_bubble, (void*)"system");
            
            // Set appropriate width for content
            lv_obj_set_width(msg_bubble, LV_SIZE_CONTENT);
            lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);
            
            // Don't grow
            lv_obj_set_style_flex_grow(msg_bubble, 0, 0);
        }
        
        // Create a full-width container for user messages to ensure right alignment
        if (strcmp(role, "user") == 0) {
            // Create a full-width container
            lv_obj_t* container = lv_obj_create(content_);
            lv_obj_set_width(container, LV_HOR_RES);
            lv_obj_set_height(container, LV_SIZE_CONTENT);
            
            // Make container transparent and borderless
            lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(container, 0, 0);
            lv_obj_set_style_pad_all(container, 0, 0);
            
            // Move the message bubble into this container
            lv_obj_set_parent(msg_bubble, container);
            
            // Right align the bubble in the container
            lv_obj_align(msg_bubble, LV_ALIGN_RIGHT_MID, -10, 0);
            
            // Auto-scroll to this container
            lv_obj_scroll_to_view_recursive(container, LV_ANIM_ON);
        } else if (strcmp(role, "system") == 0) {
            // 为系统消息创建全宽容器以确保居中对齐
            lv_obj_t* container = lv_obj_create(content_);
            lv_obj_set_width(container, LV_HOR_RES);
            lv_obj_set_height(container, LV_SIZE_CONTENT);
            
            // 使容器透明且无边框
            lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(container, 0, 0);
            lv_obj_set_style_pad_all(container, 0, 0);
            
            // 将消息气泡移入此容器
            lv_obj_set_parent(msg_bubble, container);
            
            // 将气泡居中对齐在容器中
            lv_obj_align(msg_bubble, LV_ALIGN_CENTER, 0, 0);
            
            // 自动滚动底部
            lv_obj_scroll_to_view_recursive(container, LV_ANIM_ON);
        } else {
            // For assistant messages
            // Left align assistant messages
            lv_obj_align(msg_bubble, LV_ALIGN_LEFT_MID, 0, 0);

            // Auto-scroll to the message bubble
            lv_obj_scroll_to_view_recursive(msg_bubble, LV_ANIM_ON);
        }
        
        // Store reference to the latest message label
        chat_message_label_ = msg_text;
    }
}

#else
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
    lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);

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
    lv_obj_add_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);

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
    // SpiLcdDisplay::SetupUI();
}
void XiaoziyunliaoDisplay::SetChatMessage(const char* role, const char* content) {
    DisplayLockGuard lock(this);
    SpiLcdDisplay::SetChatMessage(role, content);
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
        lv_qrcode_update(console_qrcode_, CONSOLE_URL, strlen(CONSOLE_URL));
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
    LcdDisplay::SetStatus(status);    
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
#endif


void XiaoziyunliaoDisplay::NewSmartConfigPage() {
    DisplayLockGuard lock(this);
    DelConfigPage();
    ShowChatPage();
    if (emotion_label_) {
        lv_obj_add_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
    }
#if not ((defined ja_jp) || (defined en_us))
    if (content_) {
        smartconfig_qrcode_ = lv_qrcode_create(content_);
        lv_qrcode_set_size(smartconfig_qrcode_, 100);
        lv_qrcode_set_dark_color(smartconfig_qrcode_, lv_color_white());
        lv_qrcode_set_light_color(smartconfig_qrcode_, lv_color_black());
        lv_qrcode_update(smartconfig_qrcode_, WIFI_URL, strlen(WIFI_URL));
    }
#endif
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

void XiaoziyunliaoDisplay::SetLogo(const char* logo) {
    DisplayLockGuard lock(this);
    if (logo_label_ && logo) {
        lv_label_set_text(logo_label_, logo);
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
    lv_obj_set_style_text_font(config_text_panel_, FONT2, 0);
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
#if CONFIG_BOARD_TYPE_YUNLIAO_V1
    qrcode_text += Lang::Strings::VERSION1;
#else
    qrcode_text += Lang::Strings::VERSION2;
#endif
    qrcode_text += "\n";
    qrcode_text += Lang::Strings::HELP3;
    lv_label_set_text(qrcode_label_, qrcode_text.c_str());
    lv_obj_set_style_text_font(qrcode_label_, FONT2, 0);
    lv_obj_set_style_text_line_space(qrcode_label_, 2, 0);
    lv_obj_set_style_text_align(qrcode_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(qrcode_label_, LV_PCT(100));

    // 直接创建二维码
    config_qrcode_panel_ = lv_qrcode_create(right_container);
    lv_qrcode_set_size(config_qrcode_panel_, 120);
    lv_qrcode_set_dark_color(config_qrcode_panel_, lv_color_white());
    lv_qrcode_set_light_color(config_qrcode_panel_, lv_color_black());
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
    if (emotion_label_) {
        lv_obj_add_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
    }
    if (chat_message_label_) {
        lv_obj_add_flag(chat_message_label_, LV_OBJ_FLAG_HIDDEN);
    }
}

void XiaoziyunliaoDisplay::ShowChatPage() {
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

bool XiaoziyunliaoDisplay::isWifiConfigStatus() const {
    return current_status_ == Lang::Strings::WIFI_CONFIG_MODE;
}

void XiaoziyunliaoDisplay::UpdateStatusBar(bool update_all) {
    auto& board = Board::GetInstance();
    int battery_level;
    bool charging, discharging;
    if (battery_label_ && lv_obj_has_flag(battery_label_, LV_OBJ_FLAG_HIDDEN)
        && board.GetBatteryLevel(battery_level, charging, discharging)
        && battery_level > 0) {
        lv_obj_clear_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);
    }
    SpiLcdDisplay::UpdateStatusBar(update_all);
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

// void XiaoziyunliaoDisplay::SetEmotion(const char* emotion) {
//     DisplayLockGuard lock(this);
//     if()
//     LcdDisplay::SetEmotion(emotion);
// }
