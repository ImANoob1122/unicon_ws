#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/group_sync_read.h"
#include "dynamixel_sdk/group_sync_write.h"
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "uniconlink_flex_interfaces/msg/unicon_command.hpp"
#include "uniconlink_flex_interfaces/msg/unicon_response.hpp"
#include "uniconlink_flex_interfaces/msg/variant_value.hpp"

#include "dynamixel_controller/dynamixel_controller.hpp"
#include "dynamixel_controller/msg/dynamixel_controller.hpp"

// 接続情報のマクロ（必要に応じて調整）
#define BAUDRATE 1000000
#define DEVICE_NAME "/dev/ttyUSB0"

// メッセージ定義のショートカット
#define MSG dynamixel_controller::msg::DynamixelController

// Utility functions for converting between Dynamixel data and UniconLink variant values
uniconlink_flex_interfaces::msg::VariantValue create_variant_value(const std::vector<uint8_t>& data) {
    uniconlink_flex_interfaces::msg::VariantValue var_val;
    
    if (data.size() == 1) {
        var_val.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_INT;
        var_val.int_value = static_cast<int32_t>(data[0]);
    } else if (data.size() == 2) {
        var_val.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_INT;
        var_val.int_value = static_cast<int32_t>(data[0] | (data[1] << 8));
    } else if (data.size() == 4) {
        var_val.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_INT;
        var_val.int_value = static_cast<int32_t>(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    } else {
        var_val.type = uniconlink_flex_interfaces::msg::VariantValue::TYPE_STRING;
        std::string hex_str = "";
        for (uint8_t byte : data) {
            hex_str += std::to_string(byte) + " ";
        }
        var_val.string_value = hex_str;
    }
    
    return var_val;
}

std::vector<uint8_t> extract_data_from_variant(const uniconlink_flex_interfaces::msg::VariantValue& var_val) {
    std::vector<uint8_t> data;
    
    switch (var_val.type) {
        case uniconlink_flex_interfaces::msg::VariantValue::TYPE_INT: {
            int32_t value = var_val.int_value;
            data.push_back(static_cast<uint8_t>(value & 0xFF));
            if (value > 255) {
                data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
            }
            if (value > 65535) {
                data.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
                data.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
            }
            break;
        }
        case uniconlink_flex_interfaces::msg::VariantValue::TYPE_BOOL:
            data.push_back(var_val.bool_value ? 1 : 0);
            break;
        case uniconlink_flex_interfaces::msg::VariantValue::TYPE_FLOAT: {
            uint32_t float_bits = *reinterpret_cast<const uint32_t*>(&var_val.float_value);
            data.push_back(static_cast<uint8_t>(float_bits & 0xFF));
            data.push_back(static_cast<uint8_t>((float_bits >> 8) & 0xFF));
            data.push_back(static_cast<uint8_t>((float_bits >> 16) & 0xFF));
            data.push_back(static_cast<uint8_t>((float_bits >> 24) & 0xFF));
            break;
        }
        default:
            data.push_back(0);
            break;
    }
    
    return data;
}

DynamixelController::DynamixelController() : Node("dynamixel_controller_node") {
    RCLCPP_INFO(this->get_logger(), "DynamixelController node started.");

    // ポートハンドラ、パケットハンドラの初期化
    port_handler_ = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);

    if (!port_handler_->openPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", DEVICE_NAME);
        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Port opened: %s", DEVICE_NAME);
    }

    if (!port_handler_->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate: %d", BAUDRATE);
        port_handler_->closePort();
        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Baudrate set: %d", BAUDRATE);
    }

    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

    // Load bus configuration from parameters
    std::vector<int64_t> ttl_param;
    std::vector<int64_t> rs485_param;
    this->declare_parameter("ttl_ids", ttl_param);
    this->declare_parameter("rs485_ids", rs485_param);
    this->get_parameter("ttl_ids", ttl_param);
    this->get_parameter("rs485_ids", rs485_param);
    for (auto id : ttl_param) {
        ttl_ids_.insert(static_cast<uint8_t>(id));
    }
    for (auto id : rs485_param) {
        rs485_ids_.insert(static_cast<uint8_t>(id));
    }

    // Debug: Print loaded configuration
    RCLCPP_INFO(this->get_logger(), "=== Bus Configuration Loaded ===");
    RCLCPP_INFO(this->get_logger(), "TTL IDs: ");
    for (auto id : ttl_ids_) {
        RCLCPP_INFO(this->get_logger(), "  TTL ID: %d", id);
    }
    RCLCPP_INFO(this->get_logger(), "RS485 IDs: ");
    for (auto id : rs485_ids_) {
        RCLCPP_INFO(this->get_logger(), "  RS485 ID: %d", id);
    }
    RCLCPP_INFO(this->get_logger(), "================================");

    // ROS2 サブスクライバーの作成 (送信用命令を受け付ける)
    instruction_subscriber_ = this->create_subscription<uniconlink_flex_interfaces::msg::UniconCommand>(
        "unicon_tx", 100,
        std::bind(&DynamixelController::instruction_callback, this, std::placeholders::_1));

    // ROS2 パブリッシャーの作成 (受信応答を publish する)
    response_publisher_ = this->create_publisher<uniconlink_flex_interfaces::msg::UniconResponse>("unicon_rx", 10);
}

DynamixelController::~DynamixelController() {
    // ポートを閉じ、リソース解放
    port_handler_->closePort();
    delete port_handler_;
    // packet_handler_ はシングルトンのため、削除不要の場合があります。
}

void DynamixelController::publish_response(uint8_t instruction_code, const std::vector<uint8_t> & response) {
    uniconlink_flex_interfaces::msg::UniconResponse msg;
    
    // Map instruction_code to index and create response
    msg.indices.push_back(static_cast<uint32_t>(instruction_code));
    msg.values.push_back(create_variant_value(response));
    
    response_publisher_->publish(msg);
}

void DynamixelController::instruction_callback(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
    switch (msg->command) {
        case uniconlink_flex_interfaces::msg::UniconCommand::CMD_READ: {
            handle_read_command(msg);
            break;
        }
        case uniconlink_flex_interfaces::msg::UniconCommand::CMD_WRITE: {
            handle_write_command(msg);
            break;
        }
        case uniconlink_flex_interfaces::msg::UniconCommand::CMD_SYNC_READ: {
            handle_sync_read_command(msg);
            break;
        }
        case uniconlink_flex_interfaces::msg::UniconCommand::CMD_SYNC_WRITE: {
            handle_sync_write_command(msg);
            break;
        }
        default:
            RCLCPP_WARN(this->get_logger(), "Received unsupported command: %d", msg->command);
            break;
    }
}

void DynamixelController::handle_read_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
    if (msg->indices.empty() || msg->values.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "READ command requires index for ID and two values for address and length");
        return;
    }
    
    uint8_t dxl_id = static_cast<uint8_t>(msg->indices[0]);
    uint8_t read_address = static_cast<uint8_t>(msg->values[0].int_value);
    uint8_t read_length = static_cast<uint8_t>(msg->values[1].int_value);
    uint8_t dxl_error = 0;
    int comm_result = COMM_TX_FAIL;
    std::vector<uint8_t> response_data;
    
    if (read_length == 1) {
        uint8_t data = 0;
        comm_result = packet_handler_->read1ByteTxRx(port_handler_, dxl_id, read_address, &data, &dxl_error);
        if (comm_result == COMM_SUCCESS) {
            response_data.push_back(data);
        }
    } else if (read_length == 2) {
        uint16_t data = 0;
        comm_result = packet_handler_->read2ByteTxRx(port_handler_, dxl_id, read_address, &data, &dxl_error);
        if (comm_result == COMM_SUCCESS) {
            response_data.push_back(static_cast<uint8_t>(data & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
        }
    } else if (read_length == 4) {
        uint32_t data = 0;
        comm_result = packet_handler_->read4ByteTxRx(port_handler_, dxl_id, read_address, &data, &dxl_error);
        if (comm_result == COMM_SUCCESS) {
            response_data.push_back(static_cast<uint8_t>(data & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 16) & 0xFF));
            response_data.push_back(static_cast<uint8_t>((data >> 24) & 0xFF));
        }
    }
    
    if (comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Read failed for ID %d: %s", dxl_id, packet_handler_->getTxRxResult(comm_result));
    } else {
        publish_response(uniconlink_flex_interfaces::msg::UniconCommand::CMD_READ, response_data);
    }
}

void DynamixelController::handle_write_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
    if (msg->indices.empty() || msg->values.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "WRITE command requires index for ID and at least two values");
        return;
    }
    
    uint8_t dxl_id = static_cast<uint8_t>(msg->indices[0]);
    uint8_t write_address = static_cast<uint8_t>(msg->values[0].int_value);
    uint8_t dxl_error = 0;
    int comm_result = COMM_TX_FAIL;
    
    // Extract write data from remaining values
    if (msg->values.size() == 2) {
        // Single byte write
        uint8_t value = static_cast<uint8_t>(msg->values[1].int_value);
        comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id, write_address, value, &dxl_error);
    } else if (msg->values.size() == 3) {
        // Two byte write
        uint16_t value = static_cast<uint16_t>(msg->values[1].int_value) | 
                         (static_cast<uint16_t>(msg->values[2].int_value) << 8);
        comm_result = packet_handler_->write2ByteTxRx(port_handler_, dxl_id, write_address, value, &dxl_error);
    } else if (msg->values.size() == 5) {
        // Four byte write
        uint32_t value = static_cast<uint32_t>(msg->values[1].int_value) |
                         (static_cast<uint32_t>(msg->values[2].int_value) << 8) |
                         (static_cast<uint32_t>(msg->values[3].int_value) << 16) |
                         (static_cast<uint32_t>(msg->values[4].int_value) << 24);
        comm_result = packet_handler_->write4ByteTxRx(port_handler_, dxl_id, write_address, value, &dxl_error);
    }
    
    std::vector<uint8_t> response_data = {dxl_error};
    if (comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Write failed for ID %d: %s", dxl_id, packet_handler_->getTxRxResult(comm_result));
    } else {
        publish_response(uniconlink_flex_interfaces::msg::UniconCommand::CMD_WRITE, response_data);
    }
}

void DynamixelController::handle_sync_read_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
    if (msg->indices.empty() || msg->values.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "SYNC_READ command requires indices for IDs and two values for address and length");
        return;
    }
    
    uint8_t start_address = static_cast<uint8_t>(msg->values[0].int_value);
    uint8_t data_length = static_cast<uint8_t>(msg->values[1].int_value);
    std::vector<uint8_t> id_list;
    
    for (auto index : msg->indices) {
        id_list.push_back(static_cast<uint8_t>(index));
    }
    
    std::vector<uint8_t> ttl_targets;
    std::vector<uint8_t> rs_targets;
    for (auto id : id_list) {
        if (rs485_ids_.count(id)) {
            rs_targets.push_back(id);
        } else {
            ttl_targets.push_back(id);
        }
    }
    
    std::unordered_map<uint8_t, std::vector<uint8_t>> data_map;
    uniconlink_flex_interfaces::msg::UniconResponse response_msg;
    
    // Handle TTL devices
    if (!ttl_targets.empty()) {
        dynamixel::GroupSyncRead ttlRead(port_handler_, packet_handler_, start_address, data_length);
        for (auto id : ttl_targets) {
            ttlRead.addParam(id);
        }
        int comm_result = ttlRead.txRxPacket();
        if (comm_result == COMM_SUCCESS) {
            for (auto id : ttl_targets) {
                if (ttlRead.isAvailable(id, start_address, data_length)) {
                    uint32_t data = ttlRead.getData(id, start_address, data_length);
                    std::vector<uint8_t> bytes;
                    for (uint8_t i = 0; i < data_length; i++) {
                        bytes.push_back(static_cast<uint8_t>((data >> (i*8)) & 0xFF));
                    }
                    response_msg.indices.push_back(static_cast<uint32_t>(id));
                    response_msg.values.push_back(create_variant_value(bytes));
                }
            }
        }
    }
    
    // Handle RS485 devices
    if (!rs_targets.empty()) {
        dynamixel::GroupSyncRead rsRead(port_handler_, packet_handler_, start_address, data_length);
        for (auto id : rs_targets) {
            rsRead.addParam(id);
        }
        int comm_result = rsRead.txRxPacket();
        if (comm_result == COMM_SUCCESS) {
            for (auto id : rs_targets) {
                if (rsRead.isAvailable(id, start_address, data_length)) {
                    uint32_t data = rsRead.getData(id, start_address, data_length);
                    std::vector<uint8_t> bytes;
                    for (uint8_t i = 0; i < data_length; i++) {
                        bytes.push_back(static_cast<uint8_t>((data >> (i*8)) & 0xFF));
                    }
                    response_msg.indices.push_back(static_cast<uint32_t>(id));
                    response_msg.values.push_back(create_variant_value(bytes));
                }
            }
        }
    }
    
    response_publisher_->publish(response_msg);
}

void DynamixelController::handle_sync_write_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
    if (msg->indices.empty() || msg->values.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE command requires indices and values");
        return;
    }
    
    uint8_t start_address = static_cast<uint8_t>(msg->values[0].int_value);
    uint8_t data_length = static_cast<uint8_t>(msg->values[1].int_value);
    
    if (msg->indices.size() * data_length + 2 != msg->values.size()) {
        RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE: data size mismatch");
        return;
    }
    
    dynamixel::GroupSyncWrite ttlWrite(port_handler_, packet_handler_, start_address, data_length);
    dynamixel::GroupSyncWrite rsWrite(port_handler_, packet_handler_, start_address, data_length);
    bool ttl_has_param = false;
    bool rs_has_param = false;
    
    for (size_t i = 0; i < msg->indices.size(); i++) {
        uint8_t id = static_cast<uint8_t>(msg->indices[i]);
        std::vector<uint8_t> param_data;
        
        for (uint8_t j = 0; j < data_length; j++) {
            size_t value_index = 2 + i * data_length + j;
            param_data.push_back(static_cast<uint8_t>(msg->values[value_index].int_value));
        }
        
        if (rs485_ids_.count(id)) {
            rsWrite.addParam(id, param_data.data());
            rs_has_param = true;
        } else {
            ttlWrite.addParam(id, param_data.data());
            ttl_has_param = true;
        }
    }
    
    if (ttl_has_param) {
        ttlWrite.txPacket();
    }
    
    if (rs_has_param) {
        rsWrite.txPacket();
    }
    
    std::vector<uint8_t> response_data = {0}; // Success indicator
    publish_response(uniconlink_flex_interfaces::msg::UniconCommand::CMD_SYNC_WRITE, response_data);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
