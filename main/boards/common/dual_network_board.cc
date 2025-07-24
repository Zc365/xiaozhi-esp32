#include "dual_network_board.h"
#include "application.h"
#include "display.h"
#include "assets/lang_config.h"
#include "settings.h"
#include <esp_log.h>
#include <ssid_manager.h>
#include <string.h> 

static const char *TAG = "DualNetworkBoard";

DualNetworkBoard::DualNetworkBoard(gpio_num_t ml307_tx_pin, gpio_num_t ml307_rx_pin, gpio_num_t ml307_dtr_pin, int32_t default_net_type) 
    : WifiBoard(), 
      ml307_tx_pin_(ml307_tx_pin), 
      ml307_rx_pin_(ml307_rx_pin), 
      ml307_dtr_pin_(ml307_dtr_pin) {
    
    // 从Settings加载网络类型
    network_type_ = LoadNetworkTypeFromSettings(default_net_type);
    
    // 只初始化当前网络类型对应的板卡
    InitializeCurrentBoard();
}

NetworkType DualNetworkBoard::LoadNetworkTypeFromSettings(int32_t default_net_type) {
    Settings settings("network", false);
    int network_type = settings.GetInt("type", default_net_type); 
    ESP_LOGI(TAG, "Initialize board type %d" ,network_type);
    return network_type == 1 ? NetworkType::ML307 : NetworkType::WIFI;
}

void DualNetworkBoard::SaveNetworkTypeToSettings(NetworkType type) {
    Settings settings("network", true);
    int network_type = (type == NetworkType::ML307) ? 1 : 0;
    settings.SetInt("type", network_type);
}

void DualNetworkBoard::InitializeCurrentBoard() {
    if (network_type_ == NetworkType::ML307) {
        ESP_LOGI(TAG, "Initialize ML307 board");
        current_board_ = std::make_unique<Ml307Board>(ml307_tx_pin_, ml307_rx_pin_, ml307_dtr_pin_);
    // } else {
    //     ESP_LOGI(TAG, "Initialize WiFi board");
    //     current_board_ = std::make_unique<WifiBoard>();
    }
}

void DualNetworkBoard::SwitchNetworkType() {
    auto display = GetDisplay();
    if (network_type_ == NetworkType::WIFI) {
        network_type_ = NetworkType::ML307;
        auto modem = AtModem::Detect(ml307_tx_pin_, ml307_rx_pin_, GPIO_NUM_NC , -1);
        if(modem){
            SaveNetworkTypeToSettings(NetworkType::ML307);
            display->ShowNotification(Lang::Strings::SWITCH_TO_4G_NETWORK);
        }else{
            return;
        }
    } else {
        SaveNetworkTypeToSettings(NetworkType::WIFI);
        display->ShowNotification(Lang::Strings::SWITCH_TO_WIFI_NETWORK);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    auto codec = Board::GetInstance().GetAudioCodec();
    if (codec) {
        codec->EnableOutput(false);
        codec->EnableInput(false);
    }
    auto& app = Application::GetInstance();
    app.Reboot();
}


std::string DualNetworkBoard::GetBoardType() {
    return network_type_ == NetworkType::ML307 ? 
        current_board_->GetBoardType() : 
        WifiBoard::GetBoardType();
}

void DualNetworkBoard::StartNetwork() {
    auto display = Board::GetInstance().GetDisplay();
    if (network_type_ == NetworkType::WIFI) {
        display->SetStatus(Lang::Strings::CONNECTING);
        WifiBoard::StartNetwork();
    } else {
        display->SetStatus(Lang::Strings::DETECTING_MODULE);
        current_board_->StartNetwork();
    }
}

NetworkInterface* DualNetworkBoard::GetNetwork() {
    return network_type_ == NetworkType::ML307 ? 
        current_board_->GetNetwork() : 
        WifiBoard::GetNetwork();
}
const char* DualNetworkBoard::GetNetworkStateIcon() {
    return network_type_ == NetworkType::ML307 ? 
        current_board_->GetNetworkStateIcon() : 
        WifiBoard::GetNetworkStateIcon();
}

void DualNetworkBoard::SetPowerSaveMode(bool enabled) {
    network_type_ == NetworkType::ML307 ? 
        current_board_->SetPowerSaveMode(enabled) : 
        WifiBoard::SetPowerSaveMode(enabled);
}

std::string DualNetworkBoard::GetBoardJson() {   
    return network_type_ == NetworkType::ML307 ? 
        current_board_->GetBoardJson() : 
        WifiBoard::GetBoardJson();
}

bool DualNetworkBoard::GetWifiConfigMode() {
    return network_type_ == NetworkType::WIFI && 
        WifiBoard::wifi_config_mode_;
}

void DualNetworkBoard::ClearWifiConfiguration() {
    if (network_type_ == NetworkType::WIFI) {
        auto &ssid_manager = SsidManager::GetInstance();
        ssid_manager.Clear();
        ESP_LOGI(TAG, "WiFi configuration and SSID list cleared");
        auto codec = Board::GetInstance().GetAudioCodec();
        if (codec) {
            codec->EnableOutput(false);
            codec->EnableInput(false);
        }        
        WifiBoard::ResetWifiConfiguration();
    }
}

void DualNetworkBoard::SetFactoryWifiConfiguration() {
#if defined(CONFIG_WIFI_FACTORY_SSID)
        auto &ssid_manager = SsidManager::GetInstance();
        auto ssid_list = ssid_manager.GetSsidList();
        if (strlen(CONFIG_WIFI_FACTORY_SSID) > 0){
            ssid_manager.Clear();
            ssid_manager.AddSsid(CONFIG_WIFI_FACTORY_SSID, CONFIG_WIFI_FACTORY_PASSWORD);
            Settings settings("wifi", true);
            settings.SetInt("force_ap", 0);
            auto codec = Board::GetInstance().GetAudioCodec();
            if (codec) {
                codec->EnableOutput(false);
                codec->EnableInput(false);
            }
            esp_restart();
        }
#endif
}
void DualNetworkBoard::EnterWifiConfigMode() {
    WifiBoard::EnterWifiConfigMode();
}
std::string DualNetworkBoard::GetDeviceStatusJson() {
    return network_type_ == NetworkType::ML307 ? 
        current_board_->GetDeviceStatusJson() : 
        WifiBoard::GetDeviceStatusJson();    
}