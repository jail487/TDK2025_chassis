# TDK2025 Chassis Documentation

This folder contains all documentation files and example code for the TDK2025 Chassis project.

## 📁 File Organization

### 🚗 Main Project Documentation
- `README.md` - Main project README
- `CubeMX_Step_by_Step_Guide.md` - STM32CubeMX configuration guide
- `STM32CubeMX_Configuration_Guide.md` - Detailed CubeMX setup instructions

### 📍 Pinpoint OTOS Documentation & Examples
- `PINPOINT_DRIVER_README.md` - Main Pinpoint driver documentation
- `PINPOINT_INTEGRATION_README.md` - Integration guide for chassis system
- `PINPOINT_DMA_USAGE.md` - DMA mode usage and implementation
- `PINPOINT_LIVE_EXPRESSIONS_GUIDE.md` - Debugging with Live Expressions

**📄 Text Documentation:**
- `pinpoint_communication_test.txt` - Communication test procedures
- `pinpoint_simple_test.txt` - Simple test examples
- `pinpoint_usage_example.txt` - Usage examples and code snippets

**💻 Example Code:**
- `pinpoint_example.cpp` - Basic Pinpoint usage example (C++)
- `pinpoint_example.c` - Basic Pinpoint usage example (C)
- `pinpoint_usage_example.cpp` - Advanced usage examples with chassis integration
- `pinpoint_simple_test.cpp` - Simple test implementation
- `pinpoint_communication_test.cpp` - I2C communication test code

### 📏 VL53L0X Sensor Documentation & Examples
- `VL53L0X_README.md` - Main VL53L0X sensor documentation
- `VL53L0X_Class_README.md` - C++ class implementation guide
- `VL53L0X_BUILD_FIXES.md` - Build and compilation fixes
- `VL53L0X_MODES_USAGE.md` - Different operating modes

**💻 VL53L0X Example Code:**
- `VL53L0X_Class_Example.cpp` - C++ class usage example
- `vl53l0x_example.c` - Basic VL53L0X usage example (C)
- `vl53l0x_simple.c` - Simple VL53L0X implementation
- `vl53l0x_simple.h` - Header for simple VL53L0X implementation

### 🏗️ Lifter System Documentation
- `Lifter_Update_Guide.md` - Lifter system update and usage guide

### 🤖 ROS Integration Documentation & Examples
- `ROS_DEBUG_CHECKLIST.md` - ROS debugging checklist and troubleshooting
- `ros_test_minimal.cpp` - Minimal ROS test implementation

## 📋 Quick Start Guide

1. **Hardware Setup**: Start with `STM32CubeMX_Configuration_Guide.md`
2. **Pinpoint Integration**: Follow `PINPOINT_INTEGRATION_README.md`
3. **Test Implementation**: Use `pinpoint_simple_test.cpp` as reference
4. **Advanced Usage**: Refer to `pinpoint_usage_example.cpp`
5. **ROS Integration**: Check `ros_test_minimal.cpp` for ROS examples
6. **VL53L0X Setup**: Refer to `VL53L0X_README.md`
7. **ROS Debugging**: Use `ROS_DEBUG_CHECKLIST.md` for troubleshooting

## 🔧 Development Notes

All documentation and example code have been moved to this `docs/` folder to keep the main project directory clean and organized. The example CPP files are ready-to-use code templates that can be copied back to the main project when needed.

### 📝 Using Example Code

To use any of the example CPP files:
1. Copy the desired example file from `docs/` to the appropriate project folder
2. Modify the code according to your specific requirements
3. Include it in your build configuration

---
*Last updated: September 7, 2025*
