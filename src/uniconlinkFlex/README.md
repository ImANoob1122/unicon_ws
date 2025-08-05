# UniconlinkFlex

A ROS2 package for memory-mapped I/O communication with microcontrollers using variant-based data types and synchronous read/write operations.

## Overview

UniconlinkFlex provides a flexible communication interface between ROS2 nodes and microcontroller systems through memory-mapped I/O. The system uses `std::variant` to support multiple data types (bool, int32_t, float, string) and implements both single and batch operations for efficient communication.

## Architecture

### Core Components

- **MemoryMap**: Core class managing memory-mapped variables with access control
- **VariantConverter**: Converts between C++ variants and ROS2 message types
- **UniconlinkFlexNode**: Main ROS2 node handling communication protocol

### Message Types

- **VariantValue**: Represents a single variant value with type information
- **UniconCommand**: Command message for READ/WRITE operations
- **UniconResponse**: Response message containing variable data

## Features

### Memory-Mapped I/O
- Variables stored as `std::variant<bool, int32_t, float, std::string>`
- Three access modes: `READ_ONLY`, `WRITE_ONLY`, `READ_WRITE`
- Callback support for hardware integration
- Index-based variable addressing

### Synchronous Operations
- **READ**: Single variable read
- **WRITE**: Single variable write
- **SYNC_READ**: Multiple variable read in one operation
- **SYNC_WRITE**: Multiple variable write in one operation

### Communication Topics
- `unicon_tx`: Command input topic
- `unicon_rx`: Response output topic

## Usage

### Building the Package

```bash
# Build interfaces first
colcon build --packages-select uniconlink_flex_interfaces

# Build main package
source install/setup.bash
colcon build --packages-select uniconlinkFlex

# Build example
colcon build --packages-select uniconlink_flex_example_py
```

### Running the Node

```bash
# Source the workspace
source install/setup.bash

# Run the main node
ros2 run uniconlinkFlex uniconlinkFlex_node
```

### Example Usage (Python)

```python
import rclpy
from rclpy.node import Node
from uniconlink_flex_interfaces.msg import UniconCommand, UniconResponse, VariantValue

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.unicon_tx = self.create_publisher(UniconCommand, 'unicon_tx', 10)
        self.unicon_rx = self.create_subscription(UniconResponse, 'unicon_rx', self.response_callback, 10)
    
    def response_callback(self, msg):
        # Handle responses from microcontroller
        for i, (index, value) in enumerate(zip(msg.indices, msg.values)):
            print(f"Index {index}: {self.variant_to_string(value)}")
    
    def send_read_command(self, index):
        """Read single variable"""
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_READ
        cmd.indices = [index]
        self.unicon_tx.publish(cmd)
    
    def send_sync_read_command(self, indices):
        """Read multiple variables: [0,1,3]"""
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_SYNC_READ
        cmd.indices = indices
        self.unicon_tx.publish(cmd)
    
    def send_write_command(self, index, value):
        """Write single variable"""
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_WRITE
        cmd.indices = [index]
        cmd.values = [self.create_variant_value(value)]
        self.unicon_tx.publish(cmd)
    
    def send_sync_write_command(self, indices, values):
        """Write multiple variables: [0,2,4], [0.1, "hello", 1234]"""
        cmd = UniconCommand()
        cmd.command = UniconCommand.CMD_SYNC_WRITE
        cmd.indices = indices
        cmd.values = [self.create_variant_value(v) for v in values]
        self.unicon_tx.publish(cmd)
```

### Example Commands

```python
# Single operations
node.send_read_command(0)                    # Read motor goal speed
node.send_write_command(0, 123.45)          # Write motor goal speed

# Batch operations  
node.send_sync_read_command([0, 1, 3])      # Read multiple variables
node.send_sync_write_command([0, 2, 4], [0.1, "hello", 1234])  # Write multiple
```

## Memory Map Example

The default implementation includes these example variables:

| Index | Type | Access | Description |
|-------|------|--------|-------------|
| 0 | float | READ_WRITE | Motor goal speed |
| 1 | string | READ_ONLY | Status message |
| 2 | bool | READ_WRITE | Enable flag |
| 3 | int32_t | READ_ONLY | Current position (with callback) |
| 4 | string | READ_WRITE | Device name |

## Integration with Hardware

### Microcontroller Side

The microcontroller should maintain a similar memory map structure:

```cpp
// Example microcontroller memory map
std::variant<bool, int32_t, float, std::string> memory_map[MAX_VARIABLES];

// Variable 0: Motor goal speed
memory_map[0] = 0.0f;

// Variable 1: Status
memory_map[1] = std::string("OK");

// Variable 2: Enable flag
memory_map[2] = true;

// Variable 3: Current position
memory_map[3] = 100;

// Variable 4: Device name
memory_map[4] = std::string("device1");
```

### Callback Integration

```cpp
// Example callback for hardware integration
memory_map_->set_variable(0, 0.0f, AccessMode::READ_WRITE,
    [this](const VariantType& value) {
        // Write callback - update actual hardware
        float speed = std::get<float>(value);
        motor_controller.set_goal_speed(speed);
    },
    [this]() -> VariantType {
        // Read callback - get current hardware state
        return motor_controller.get_goal_speed();
    });
```

## Communication Protocol

### Request Format (unicon_tx)
```
UniconCommand {
    uint8 command       // CMD_READ, CMD_WRITE, CMD_SYNC_READ, CMD_SYNC_WRITE
    uint32[] indices    // Variable indices to operate on
    VariantValue[] values // Values for write operations
}
```

### Response Format (unicon_rx)
```
UniconResponse {
    uint32[] indices    // Variable indices
    VariantValue[] values // Current values
}
```

### Periodic Updates

The node automatically publishes periodic updates from the microcontroller in the format:
```
[[0, 1, 2], [100, "hogehoge", 0.33]]
```

## Running the Example

```bash
# Terminal 1: Start the main node
ros2 run uniconlinkFlex uniconlinkFlex_node

# Terminal 2: Run the Python example
ros2 run uniconlink_flex_example_py uniconlink_flex_example

# Terminal 3: Monitor communication
ros2 topic echo /unicon_rx
```

## Dependencies

- ROS2 Humble
- rclcpp
- std_msgs
- uniconlink_flex_interfaces (custom message package)

## License

TODO: License declaration

## Contributing

This package is part of the ROS2 workspace for robotic control systems. Please follow ROS2 coding standards and conventions when contributing.