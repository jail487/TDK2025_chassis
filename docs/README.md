# TDK2025 Chassis

This project is for the TDK2025 chassis, which includes communication, localization, and control for a robotic system. It is developed using STM32CubeIDE and integrates ROS1 for communication.

## Features
- **ROS1 Communication**: Publishes and subscribes to ROS topics for controlling the chassis and receiving mission updates.
- **Localization**: Tracks the chassis position and orientation in real-time.
- **Motor Control**: Implements inverse and forward kinematics for mecanum wheels.
- **VL53L0X Sensor Integration**: Advanced Time-of-Flight sensors for distance measurement and obstacle detection.
- **Lifter Control**: Precise positioning using VL53L0X feedback sensors.

## Folder Structure
- `ROS1/`: Contains ROS communication code.
- `Chassis/`: Handles motor control and localization.
- `location/`: Implements pathfinding, movement logic, and VL53L0X sensor drivers.
- `Lifter/`: Controls the lifter mechanism with VL53L0X position feedback.
- `Drivers/VL53L0X/`: Official STMicroelectronics VL53L0X API.

## VL53L0X Sensor System

### Overview
The project includes a comprehensive VL53L0X Time-of-Flight sensor integration with:

1. **Official ST API** (`Drivers/VL53L0X/`): Complete STMicroelectronics implementation
2. **Enhanced C++ Classes** (`location/VL53L0X_Class.h/cpp`): Object-oriented sensor management
3. **Practical Integration** (`Lifter/`): Real-world application in lifter control

### Quick Start - Using the Enhanced C++ Class

```cpp
#include "VL53L0X_Class.h"

// Create sensor instance
VL53L0X_Enhanced sensor(GPIOA, GPIO_PIN_8, 0x30, &hi2c1, "Front_Sensor");

// Initialize with high accuracy
if (sensor.Init(VL53_Accuracy::HIGH)) {
    // Take measurement
    uint16_t distance = sensor.GetDistance();
    
    // Check if valid
    if (sensor.IsLastMeasurementValid()) {
        printf("Distance: %d mm\n", distance);
    }
    
    // Or get detailed measurement
    VL53_MeasurementData data;
    if (sensor.GetMeasurement(data)) {
        printf("Distance: %d mm, Signal: %d, Valid: %s\n",
               data.distance_mm, data.signal_rate,
               data.is_valid ? "Yes" : "No");
    }
}
```

### Multi-Sensor Setup

```cpp
// Define multiple sensors with unique addresses
VL53L0X_Enhanced front_sensor(GPIOA, GPIO_PIN_8, 0x30, &hi2c1, "Front");
VL53L0X_Enhanced back_sensor(GPIOA, GPIO_PIN_9, 0x31, &hi2c1, "Back");

// Initialize all sensors
if (front_sensor.Init() && back_sensor.Init()) {
    // Use operator overloads for easy comparison
    if (front_sensor < 100) {
        printf("Obstacle detected in front!\n");
    }
    
    if (back_sensor > 200) {
        printf("Clear behind\n");
    }
}
```

### Calibration

```cpp
// Perform comprehensive calibration
bool success = sensor.PerformFullCalibration(100); // 100mm target distance

// Or individual calibrations
sensor.CalibrateOffset(100);      // Offset calibration
sensor.CalibrateCrosstalk();      // Crosstalk calibration
```

### Available Accuracy Modes

- **VL53_Accuracy::FAST**: Quick measurements, lower accuracy
- **VL53_Accuracy::BETTER**: Balanced speed and accuracy  
- **VL53_Accuracy::HIGH**: Maximum accuracy, slower measurements

### Integration in Lifter System

The lifter system uses VL53L0X sensors for precise position feedback:

```cpp
// From lifter.cpp - Enhanced integration
VL53L0X_Enhanced front_sensor(GPIOA, GPIO_PIN_8, 0x52, &hi2c1, "Front_Lifter");
VL53L0X_Enhanced back_sensor(GPIOA, GPIO_PIN_9, 0x54, &hi2c1, "Back_Lifter");

void lifterSetup() {
    // Initialize sensors with high accuracy for precise positioning
    bool front_ok = front_sensor.Init(VL53_Accuracy::HIGH, 50000);
    bool back_ok = back_sensor.Init(VL53_Accuracy::HIGH, 50000);
    
    if (front_ok && back_ok) {
        // Calibrate sensors for improved accuracy
        calibrateLifterSensors(100);
    }
}

uint16_t getLifterPosition(int lifter) {
    VL53L0X_Enhanced& sensor = (lifter == 0) ? front_sensor : back_sensor;
    
    if (sensor.IsInitialized()) {
        uint16_t distance = sensor.GetDistance();
        return sensor.IsLastMeasurementValid() ? distance : 0;
    }
    return 0;
}
```

### Error Handling

The system provides comprehensive error handling:

```cpp
// Check sensor status
if (!sensor.IsInitialized()) {
    printf("Sensor not initialized\n");
}

// Get last error
VL53_Status status = sensor.GetLastStatus();
switch (status) {
    case VL53_Status::VALID:
        printf("Measurement valid\n");
        break;
    case VL53_Status::OUT_OF_RANGE:
        printf("Target out of range\n");
        break;
    case VL53_Status::SIGNAL_TOO_LOW:
        printf("Signal too weak\n");
        break;
    // ... handle other cases
}
```

### Debugging and Monitoring

```cpp
// Print detailed sensor information
sensor.PrintStatus();
sensor.PrintCalibrationData();

// Monitor sensor performance
VL53_MeasurementData data;
if (sensor.GetMeasurement(data)) {
    printf("Time: %lu, Distance: %d, Signal: %d, Ambient: %d\n",
           data.measurement_time, data.distance_mm, 
           data.signal_rate, data.ambient_rate);
}
```

### Build Instructions

### Prerequisites
- STM32CubeIDE 1.18.1 or later
- STM32H723ZG MCU support package
- HAL library configured for I2C, GPIO, and timers

### Compilation Notes
The VL53L0X integration has been carefully designed to avoid C/C++ linkage conflicts:

- C++ headers are included outside `extern "C"` blocks
- Proper include paths are configured for cross-folder dependencies
- Operator overloads use explicit casting to avoid ambiguity

### Build Command
```bash
make -j8 all
```

### Troubleshooting Build Issues
If you encounter compilation errors:

1. **Include Path Errors**: Ensure VL53L0X API files are in `Drivers/VL53L0X/`
2. **Linkage Errors**: Check that C++ includes are outside `extern "C"` blocks

## Hardware Configuration

### VL53L0X Connections
- **VCC**: 3.3V
- **GND**: Ground
- **SDA**: I2C Data (hi2c1)
- **SCL**: I2C Clock (hi2c1)
- **XSHUT**: GPIO pin (for address programming)

### Multi-Sensor Setup
Each sensor requires a unique GPIO pin for XSHUT control:
- Front Sensor: GPIOA Pin 8
- Back Sensor: GPIOA Pin 9
- Additional sensors: Use available GPIO pins

### I2C Configuration
- **Frequency**: 400kHz recommended
- **Address Range**: 0x29-0x7F (avoid conflicts)
- **Pull-ups**: Required on SDA/SCL lines

## API Reference

### VL53L0X_Enhanced Class Methods

#### Initialization
- `Init()`: Initialize with default settings
- `Init(accuracy, timing_budget)`: Initialize with custom settings
- `IsInitialized()`: Check initialization status

#### Measurement
- `GetDistance()`: Quick distance reading
- `GetDistance(distance, status)`: Distance with status
- `GetMeasurement(data)`: Complete measurement data
- `IsLastMeasurementValid()`: Validate last reading

#### Calibration
- `CalibrateOffset(target_distance)`: Offset calibration
- `CalibrateCrosstalk()`: Crosstalk calibration
- `PerformFullCalibration(target_distance)`: Complete calibration

#### Configuration
- `SetAccuracyMode(mode)`: Change accuracy mode
- `SetTimingBudget(budget_us)`: Timing budget configuration
- `SetAddress(new_address)`: I2C address change

#### Utilities
- `PrintStatus()`: Debug information
- `PrintCalibrationData()`: Calibration values
- `GetLastStatus()`: Last measurement status
- `GetLastDistance()`: Last distance reading

#### Operator Overloads
- `operator uint16_t()`: Implicit distance conversion
- `operator>(distance)`, `operator<(distance)`: Distance comparison
- `operator==(distance)`, `operator!=(distance)`: Distance equality

## How to Use
1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/TDK2025_chassis.git
   ```

2. Open the project in STM32CubeIDE

3. Configure hardware connections according to the hardware configuration section

4. Build and flash to the STM32H723ZG microcontroller

5. Initialize VL53L0X sensors and start the system

## Contributing
When adding new features or fixing bugs, please:
1. Follow the existing code structure
2. Document new functions and classes
3. Test with both single and multi-sensor configurations
4. Update this README if adding new functionality

## License
This project is part of the TDK2025 robotics competition entry.

---

For detailed technical implementation, refer to the code comments and examples in the respective source files.