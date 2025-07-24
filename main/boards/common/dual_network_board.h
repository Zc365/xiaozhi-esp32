#ifndef DUAL_NETWORK_BOARD_H
#define DUAL_NETWORK_BOARD_H

#include "board.h"
#include "wifi_board.h"
#include "ml307_board.h"
#include "web_socket.h"
// #include "ml307_at_modem.h"
#include <memory>

//enum NetworkType
enum class NetworkType {
    WIFI,
    ML307
};

// 双网络板卡类，可以在WiFi和ML307之间切换
class DualNetworkBoard : public WifiBoard {
private:
    std::unique_ptr<Ml307Board> current_board_;  // 仅用于ML307模式
    NetworkType network_type_ = NetworkType::WIFI;  // Default to WiFi

    // ML307的引脚配置
    gpio_num_t ml307_tx_pin_;
    gpio_num_t ml307_rx_pin_;
    gpio_num_t ml307_dtr_pin_;
    
    // 从Settings加载网络类型
    NetworkType LoadNetworkTypeFromSettings(int32_t default_net_type);
    
    // 保存网络类型到Settings
    void SaveNetworkTypeToSettings(NetworkType type);

    // 初始化当前网络类型对应的板卡
    void InitializeCurrentBoard();
 
public:
    DualNetworkBoard(gpio_num_t ml307_tx_pin, gpio_num_t ml307_rx_pin, gpio_num_t ml307_dtr_pin = GPIO_NUM_NC, int32_t default_net_type = 0);
    virtual ~DualNetworkBoard() = default;
 
    // 切换网络类型
    void SwitchNetworkType();
    
    // 获取当前网络类型
    NetworkType GetNetworkType() const { return network_type_; }
    
   
    // 重写Board接口
    virtual std::string GetBoardType() override;
    virtual void StartNetwork() override;
    virtual NetworkInterface* GetNetwork() override;
    virtual const char* GetNetworkStateIcon() override;
    virtual void SetPowerSaveMode(bool enabled) override;
    virtual std::string GetBoardJson() override;
    // void ResetWifiConfiguration();
    void ClearWifiConfiguration();
    void SetFactoryWifiConfiguration();
    bool GetWifiConfigMode();
    virtual void EnterWifiConfigMode() override;
    virtual std::string GetDeviceStatusJson() override;
};

#endif // DUAL_NETWORK_BOARD_H 