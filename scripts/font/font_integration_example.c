/**
 * ESP32 LVGL 中文字体集成示例
 * 
 * 本文件展示如何在ESP32项目中使用生成的中文字体
 * 适用于使用 generate_puhui_font.py 或 generate_common_chinese_font.py 生成的字体
 */

#include "lvgl.h"

// 包含生成的字体头文件
#include "font_puhui.h"              // 完整普惠字体
// 或者
// #include "common_chinese_fonts.h"     // 常用中文字体

/**
 * 字体使用示例1: 创建带中文的标签
 */
void create_chinese_label_example(lv_obj_t *parent)
{
    // 创建标签
    lv_obj_t *label = lv_label_create(parent);
    
    // 设置中文文本
    lv_label_set_text(label, "你好世界！\n欢迎使用ESP32");
    
    // 设置字体 - 使用16像素普惠字体
    lv_obj_set_style_text_font(label, &font_puhui_16_1, 0);
    // 或者使用常用中文字体
    // lv_obj_set_style_text_font(label, &font_chinese_common_16_1, 0);
    
    // 设置位置
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    
    // 设置文本颜色
    lv_obj_set_style_text_color(label, lv_color_hex(0x000000), 0);
}

/**
 * 字体使用示例2: 创建样式并应用字体
 */
void create_styled_chinese_text(lv_obj_t *parent)
{
    // 创建样式
    static lv_style_t style_chinese;
    lv_style_init(&style_chinese);
    
    // 设置字体
    lv_style_set_text_font(&style_chinese, &font_puhui_18_1);
    
    // 设置文本颜色
    lv_style_set_text_color(&style_chinese, lv_color_hex(0x333333));
    
    // 设置文本对齐
    lv_style_set_text_align(&style_chinese, LV_TEXT_ALIGN_CENTER);
    
    // 创建标签并应用样式
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_add_style(label, &style_chinese, 0);
    
    // 设置长文本
    lv_label_set_text(label, 
        "这是一个中文字体测试\n"
        "包含常用汉字：\n"
        "你好、世界、欢迎、使用\n"
        "数字：1234567890\n"
        "符号：！@#￥%……&*（）");
    
    // 设置长文本模式
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    
    // 设置宽度
    lv_obj_set_width(label, 200);
    
    // 居中显示
    lv_obj_center(label);
}

/**
 * 字体使用示例3: 按钮中的中文文本
 */
void create_chinese_button(lv_obj_t *parent)
{
    // 创建按钮
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 120, 50);
    lv_obj_center(btn);
    
    // 创建按钮标签
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "确认");
    
    // 设置按钮标签字体
    lv_obj_set_style_text_font(label, &font_puhui_16_1, 0);
    
    // 居中显示标签
    lv_obj_center(label);
}

/**
 * 字体使用示例4: 不同尺寸字体的混合使用
 */
void create_mixed_size_text(lv_obj_t *parent)
{
    // 标题 - 使用20像素字体
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, "系统设置");
    lv_obj_set_style_text_font(title, &font_puhui_20_1, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);
    
    // 副标题 - 使用18像素字体
    lv_obj_t *subtitle = lv_label_create(parent);
    lv_label_set_text(subtitle, "网络配置");
    lv_obj_set_style_text_font(subtitle, &font_puhui_18_1, 0);
    lv_obj_align_to(subtitle, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
    
    // 内容 - 使用16像素字体
    lv_obj_t *content = lv_label_create(parent);
    lv_label_set_text(content, "WiFi名称：我的网络\n密码：********");
    lv_obj_set_style_text_font(content, &font_puhui_16_1, 0);
    lv_obj_align_to(content, subtitle, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    
    // 说明文字 - 使用14像素字体
    lv_obj_t *note = lv_label_create(parent);
    lv_label_set_text(note, "注：修改设置后需要重启设备");
    lv_obj_set_style_text_font(note, &font_puhui_14_1, 0);
    lv_obj_set_style_text_color(note, lv_color_hex(0x888888), 0);
    lv_obj_align_to(note, content, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
}

/**
 * 字体使用示例5: 文本区域中的中文输入
 */
void create_chinese_textarea(lv_obj_t *parent)
{
    // 创建文本区域
    lv_obj_t *ta = lv_textarea_create(parent);
    lv_obj_set_size(ta, 200, 100);
    lv_obj_center(ta);
    
    // 设置字体
    lv_obj_set_style_text_font(ta, &font_puhui_16_1, 0);
    
    // 设置占位符文本
    lv_textarea_set_placeholder_text(ta, "请输入中文...");
    
    // 设置默认文本
    lv_textarea_set_text(ta, "这里可以输入中文");
    
    // 设置单行模式
    lv_textarea_set_one_line(ta, false);
}

/**
 * 主函数示例：初始化并创建UI
 */
void app_main(void)
{
    // 初始化LVGL (这里省略具体的初始化代码)
    // lv_init();
    // ... 显示驱动初始化 ...
    
    // 创建主屏幕
    lv_obj_t *scr = lv_scr_act();
    
    // 设置背景颜色
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF), 0);
    
    // 根据需要调用不同的示例函数
    create_chinese_label_example(scr);
    // create_styled_chinese_text(scr);
    // create_chinese_button(scr);
    // create_mixed_size_text(scr);
    // create_chinese_textarea(scr);
    
    // 启动LVGL任务循环
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * CMakeLists.txt 配置示例：
 * 
 * idf_component_register(
 *     SRCS 
 *         "main.c"
 *         "font_puhui_14_1.c"     # 添加字体文件
 *         "font_puhui_16_1.c"
 *         "font_puhui_18_1.c"
 *         "font_puhui_20_1.c"
 *         # 或者使用常用中文字体
 *         # "font_chinese_common_14_1.c"
 *         # "font_chinese_common_16_1.c"
 *         # "font_chinese_common_18_1.c"
 *         # "font_chinese_common_20_1.c"
 *     INCLUDE_DIRS 
 *         "."
 *     REQUIRES 
 *         "lvgl"
 *         "esp_lcd"
 *         "esp_timer"
 * )
 */

/**
 * 字体选择建议：
 * 
 * 1. 内存充足的项目：
 *    - 使用完整普惠字体 (font_puhui_xx_1.c)
 *    - 支持所有常见中文字符
 *    - 文件较大 (每个尺寸约800KB-1.6MB)
 * 
 * 2. 内存受限的项目：
 *    - 使用常用中文字体 (font_chinese_common_xx_1.c)
 *    - 仅包含最常用的1000+个中文字符
 *    - 文件较小 (每个尺寸约40KB-80KB)
 * 
 * 3. 字体尺寸选择：
 *    - 14px: 适合小屏幕或辅助信息
 *    - 16px: 适合正文内容
 *    - 18px: 适合标题或重要信息
 *    - 20px: 适合大标题
 * 
 * 4. 位深选择：
 *    - 1bpp: 文件最小，黑白显示
 *    - 4bpp: 文件较大，支持16级灰度，显示效果更好
 */