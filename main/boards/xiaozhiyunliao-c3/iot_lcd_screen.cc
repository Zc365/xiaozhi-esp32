#include "board.h"
#include "iot/thing.h"
#include "display/display.h"
#include "boards/xiaozhiyunliao-c3/xiaoziyunliao_display.h"
#include <esp_log.h>

#define TAG "LCDScreen"

namespace iot {

class LCDScreen : public Thing {
public:
    LCDScreen() : Thing("LCDScreen", "当前 AI 机器人的屏幕") {

        properties_.AddNumberProperty("brightness", "当前屏幕背光亮度百分比", [this]() -> int {
            auto display = Board::GetInstance().GetDisplay();
            if (display) {
                auto lcd_display = static_cast<XiaoziyunliaoDisplay*>(display);
                ESP_LOGD(TAG, "当前背光亮度: %d%%", lcd_display->brightness());
                return lcd_display->brightness();
            }
            return 0;
        });

        methods_.AddMethod("SetBrightness", "设置屏幕背光亮度", ParameterList({
            Parameter("brightness", "0到100之间的整数", kValueTypeNumber, true)
        }), [this](const ParameterList& parameters) {
            auto display = Board::GetInstance().GetDisplay();
            if (display) {
                auto lcd_display = static_cast<XiaoziyunliaoDisplay*>(display);
                lcd_display->SetBacklight(static_cast<uint8_t>(parameters["brightness"].number()));
            }
        });

        methods_.AddMethod("ShowHelpPage", "显示帮助页面", ParameterList(), [this](const ParameterList& parameters) {
            auto display = Board::GetInstance().GetDisplay();
            if (display) {
                auto lcd_display = static_cast<XiaoziyunliaoDisplay*>(display);
                lcd_display->SwitchPage(XiaoziyunliaoDisplay::PageIndex::PAGE_CONFIG);
            }
        });

        methods_.AddMethod("ShowChatPage", "显示聊天页面", ParameterList(), [this](const ParameterList& parameters) {
            auto display = Board::GetInstance().GetDisplay();
            if (display) {
                auto lcd_display = static_cast<XiaoziyunliaoDisplay*>(display);
                lcd_display->SwitchPage(XiaoziyunliaoDisplay::PageIndex::PAGE_CHAT);
            }
        });
    }
};

} // namespace iot

DECLARE_THING(LCDScreen);




