#ifndef DYNAMIXEL_CONTROLLER_HPP_
#define DYNAMIXEL_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "uniconlink_flex_interfaces/msg/unicon_command.hpp"
#include "uniconlink_flex_interfaces/msg/unicon_response.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

class DynamixelController : public rclcpp::Node {
public:
    DynamixelController();
    ~DynamixelController();

private:
    // Instruction callback: ROS2 で受信した命令に応じた処理を実行する
    void instruction_callback(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg);
    // 指定命令に対する応答データを publish する
    void publish_response(uint8_t instruction_code, const std::vector<uint8_t> & response);
    
    // Command handlers for UniconLink protocol
    void handle_read_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg);
    void handle_write_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg);
    void handle_sync_read_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg);
    void handle_sync_write_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg);

    // ROS インターフェース
    rclcpp::Subscription<uniconlink_flex_interfaces::msg::UniconCommand>::SharedPtr instruction_subscriber_;
    rclcpp::Publisher<uniconlink_flex_interfaces::msg::UniconResponse>::SharedPtr response_publisher_;

    // Dynamixel SDK 用オブジェクト
    dynamixel::PortHandler   *port_handler_;
    dynamixel::PacketHandler *packet_handler_;

    // Bus configuration
    std::unordered_set<uint8_t> ttl_ids_;
    std::unordered_set<uint8_t> rs485_ids_;

    // デバイスパラメータ（例：モータID、プロトコルバージョン）
    const int dxl_id_ = 1;
    const double protocol_version_ = 2.0;
};

#endif // DYNAMIXEL_CONTROLLER_HPP_
