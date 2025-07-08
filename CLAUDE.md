# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 workspace containing multiple packages for robotic control systems, including:
- **Dynamixel Controller**: A ROS2 node for controlling Dynamixel servo motors
- **RogiLink Flex**: A versatile communication library for UART-based device interaction
- **STM32 HAL Integration**: Embedded firmware for STM32F4 microcontrollers with motor control capabilities

## Build System

This workspace uses ROS2 Humble with colcon as the build system.

### Basic Build Commands
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select dynamixel_controller

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
rm -rf build/ install/ log/
colcon build
```

### Environment Setup
```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source workspace
source install/local_setup.bash
```

## Package Structure

### dynamixel_controller
Location: `src/dynamixel_controller/`

Controls Dynamixel servo motors through DynamixelSDK. Key features:
- Supports PING, READ_DATA, WRITE_DATA, SYNC_READ, SYNC_WRITE commands
- Communication via `dynamixel_tx` and `dynamixel_rx` topics
- Bus configuration through `config/bus_config.yaml`

**Run the node:**
```bash
ros2 run dynamixel_controller dynamixel_controller_node --ros-args --params-file config/bus_config.yaml
```

### rogilink_flex
Location: `src/rogilinkFlex-ros2/`

Generic communication library for UART devices with automatic data conversion and device management.

**Components:**
- `rogilink_flex`: Core C++ library with UART communication
- `rogilink_flex_interfaces`: Message and service definitions
- `rogilink_flex_gui`: GUI application for communication testing
- `rogilink_flex_example_py`: Python usage examples

**Launch GUI:**
```bash
ros2 launch rogilink_flex_gui gui.launch.py
```

### STM32 HAL Code
Location: `hal_ws/ilias2025_differential_swerve_tester_hal/`

Embedded firmware for STM32F446RE microcontroller featuring:
- Motor control using DC motors and servo motors
- UART communication with RogiLink protocol
- Timer-based PWM generation
- Custom motor control libraries

**Build (requires STM32 toolchain):**
```bash
cd hal_ws/ilias2025_differential_swerve_tester_hal/
cmake --preset default
cmake --build build
```

## Development Workflow

### Testing
No specific test framework configured. Test individual nodes manually:

```bash
# Test dynamixel controller
ros2 topic echo /dynamixel_rx

# Test rogilink communication
ros2 topic echo /rogilink_flex_frame
```

### Code Organization
- C++ code follows ROS2 conventions with ament_cmake
- Python packages use standard ROS2 Python structure
- STM32 code uses HAL library with custom abstractions

### Communication Patterns
- **Dynamixel**: Uses UInt8MultiArray messages on specific topics
- **RogiLink**: Uses custom Frame messages with JSON configuration
- **STM32**: Communicates via UART with custom protocol

## Key Files
- `src/dynamixel_controller/src/dynamixel_controller.cpp`: Main Dynamixel node implementation
- `src/rogilinkFlex-ros2/rogilink_flex/src/uart_node.cpp`: UART communication node
- `hal_ws/ilias2025_differential_swerve_tester_hal/Src/main.cpp`: STM32 main application
- `src/dynamixel_controller/config/bus_config.yaml`: Dynamixel device configuration

## Development with Docker

This repository includes Docker and devcontainer configuration for development.

### Prerequisites
- Docker installed on your system
- VS Code with Dev Containers extension (optional)
- Set `CLAUDE_API_KEY` environment variable

### Using Dev Containers (Recommended)
1. Open the repository in VS Code
2. Set your Claude API key:
   ```bash
   export CLAUDE_API_KEY="your_claude_api_key_here"
   ```
3. Press `Ctrl+Shift+P` and select "Dev Containers: Reopen in Container"
4. The container will build with ROS2 Humble and Claude Code pre-installed

### Manual Docker Usage
```bash
# Build the Docker image
docker build -t unicon-ros2 .devcontainer/

# Run the container
docker run -it --rm \
  --network=host \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  -v $(pwd):/home/ros/ros2_ws \
  -e CLAUDE_API_KEY="your_claude_api_key_here" \
  unicon-ros2
```

### Using Claude Code in Container
Once inside the container:
```bash
# Initialize Claude Code (if not already done)
claude auth login

# Use Claude Code for development
claude --help
```

## Dependencies
- ROS2 Humble
- DynamixelSDK
- nlohmann/json (for RogiLink)
- STM32 HAL libraries (for embedded code)
- CppLinuxSerial (for UART communication)
- Claude Code CLI (installed in container)

## Hardware Configuration
- Dynamixel servos: Connected via USB/TTL or RS485
- STM32F446RE: Uses UART2 for communication, TIM1-7 for PWM/timing
- Default UART settings: 1000000 baud, /dev/ttyUSB0
- Serial devices: /dev/ttyUSB0-3, /dev/ttyACM0-1 (mapped in container)