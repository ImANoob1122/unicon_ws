#include "rclcpp/rclcpp.hpp"
#include "uniconlink_flex_interfaces/msg/unicon_command.hpp"
#include "uniconlink_flex_interfaces/msg/unicon_response.hpp"
#include "uniconlinkFlex/memory_map.hpp"
#include "uniconlinkFlex/variant_converter.hpp"
#include <memory>

using namespace uniconlink_flex;

class UniconlinkFlexNode : public rclcpp::Node {
public:
    UniconlinkFlexNode() : Node("uniconlink_flex_node") {
        memory_map_ = std::make_unique<MemoryMap>();
        
        // Create publishers and subscribers
        response_publisher_ = this->create_publisher<uniconlink_flex_interfaces::msg::UniconResponse>(
            "unicon_rx", 10);
        
        command_subscriber_ = this->create_subscription<uniconlink_flex_interfaces::msg::UniconCommand>(
            "unicon_tx", 10, std::bind(&UniconlinkFlexNode::command_callback, this, std::placeholders::_1));
        
        // Initialize example memory variables
        initialize_memory_variables();
        
        // Create timer for periodic data publishing
        periodic_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&UniconlinkFlexNode::publish_periodic_data, this));
        
        RCLCPP_INFO(this->get_logger(), "UniconlinkFlex node initialized");
    }
    
    MemoryMap* get_memory_map() { return memory_map_.get(); }
    
private:
    void initialize_memory_variables() {
        // Example: Index 0 - Motor goal speed (Read/Write)
        memory_map_->set_variable(0, 0.0f, AccessMode::READ_WRITE,
            [this](const VariantType& value) {
                RCLCPP_INFO(this->get_logger(), "Motor goal speed set to: %f", std::get<float>(value));
            });
        
        // Example: Index 1 - Status string (Read only)
        memory_map_->set_variable(1, std::string("OK"), AccessMode::READ_ONLY);
        
        // Example: Index 2 - Enable flag (Read/Write)
        memory_map_->set_variable(2, true, AccessMode::READ_WRITE,
            [this](const VariantType& value) {
                RCLCPP_INFO(this->get_logger(), "Enable flag set to: %s", 
                           std::get<bool>(value) ? "true" : "false");
            });
        
        // Example: Index 3 - Current position (Read only with callback)
        memory_map_->set_variable(3, 100, AccessMode::READ_ONLY, nullptr,
            [this]() -> VariantType {
                // Simulate reading from hardware
                return static_cast<int32_t>(123 + rand() % 100);
            });
        
        // Example: Index 4 - Device name (Read/Write)
        memory_map_->set_variable(4, std::string("device1"), AccessMode::READ_WRITE);
    }
    
    void command_callback(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
        switch (msg->command) {
            case uniconlink_flex_interfaces::msg::UniconCommand::CMD_READ:
                handle_read_command(msg);
                break;
            case uniconlink_flex_interfaces::msg::UniconCommand::CMD_WRITE:
                handle_write_command(msg);
                break;
            case uniconlink_flex_interfaces::msg::UniconCommand::CMD_SYNC_READ:
                handle_sync_read_command(msg);
                break;
            case uniconlink_flex_interfaces::msg::UniconCommand::CMD_SYNC_WRITE:
                handle_sync_write_command(msg);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command: %d", msg->command);
        }
    }
    
    void handle_read_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
        if (msg->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "READ command with empty indices");
            return;
        }
        
        uint32_t index = msg->indices[0];
        
        try {
            VariantType value = memory_map_->read_variable(index);
            
            auto response = uniconlink_flex_interfaces::msg::UniconResponse();
            response.indices = {index};
            response.values = {VariantConverter::to_msg(value)};
            
            response_publisher_->publish(response);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading variable %d: %s", index, e.what());
        }
    }
    
    void handle_write_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
        if (msg->indices.empty() || msg->values.empty()) {
            RCLCPP_WARN(this->get_logger(), "WRITE command with empty indices or values");
            return;
        }
        
        uint32_t index = msg->indices[0];
        VariantType value = VariantConverter::from_msg(msg->values[0]);
        
        if (memory_map_->write_variable(index, value)) {
            RCLCPP_INFO(this->get_logger(), "Successfully wrote to variable %d", index);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to variable %d", index);
        }
    }
    
    void handle_sync_read_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
        if (msg->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "SYNC_READ command with empty indices");
            return;
        }
        
        try {
            std::vector<VariantType> values = memory_map_->read_multiple(msg->indices);
            
            auto response = uniconlink_flex_interfaces::msg::UniconResponse();
            response.indices = msg->indices;
            response.values = VariantConverter::to_msg_vector(values);
            
            response_publisher_->publish(response);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in SYNC_READ: %s", e.what());
        }
    }
    
    void handle_sync_write_command(const uniconlink_flex_interfaces::msg::UniconCommand::SharedPtr msg) {
        if (msg->indices.empty() || msg->values.empty()) {
            RCLCPP_WARN(this->get_logger(), "SYNC_WRITE command with empty indices or values");
            return;
        }
        
        if (msg->indices.size() != msg->values.size()) {
            RCLCPP_ERROR(this->get_logger(), "SYNC_WRITE: indices and values size mismatch");
            return;
        }
        
        std::vector<VariantType> values = VariantConverter::from_msg_vector(msg->values);
        
        if (memory_map_->write_multiple(msg->indices, values)) {
            RCLCPP_INFO(this->get_logger(), "Successfully wrote %zu variables", msg->indices.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to write some variables in SYNC_WRITE");
        }
    }
    
    void publish_periodic_data() {
        // Simulate periodic data from microcontroller
        std::vector<uint32_t> indices = {0, 1, 3};
        
        try {
            std::vector<VariantType> values = memory_map_->read_multiple(indices);
            
            auto response = uniconlink_flex_interfaces::msg::UniconResponse();
            response.indices = indices;
            response.values = VariantConverter::to_msg_vector(values);
            
            response_publisher_->publish(response);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in periodic data publish: %s", e.what());
        }
    }
    
    std::unique_ptr<MemoryMap> memory_map_;
    rclcpp::Publisher<uniconlink_flex_interfaces::msg::UniconResponse>::SharedPtr response_publisher_;
    rclcpp::Subscription<uniconlink_flex_interfaces::msg::UniconCommand>::SharedPtr command_subscriber_;
    rclcpp::TimerBase::SharedPtr periodic_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<UniconlinkFlexNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}