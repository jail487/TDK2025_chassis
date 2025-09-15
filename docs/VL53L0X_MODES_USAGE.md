# VL53L0X Accuracy Modes Usage Guide

## Overview
The VL53L0X_Enhanced class provides 5 different accuracy modes to optimize performance for different applications. Each mode offers different trade-offs between measurement speed and precision.

## Available Modes

| Mode | Timing | Accuracy | Use Case |
|------|--------|----------|----------|
| `ULTRA_FAST` | ~13ms | ±5% | Motion detection, rapid scanning |
| `FAST` | ~20ms | ±3% | Obstacle avoidance, real-time navigation |
| `BETTER` | ~33ms | ±1% | General purpose, balanced performance |
| `HIGH` | ~200ms | ±0.5% | Precise positioning, measurement verification |
| `LONG_RANGE` | ~300ms | ±1% | Extended detection, large area monitoring |

## Usage Examples

### 1. Quick Mode Setting with Predefined Configurations

```cpp
#include "VL53L0X_Class.h"

VL53L0X_Enhanced sensor(GPIOA, GPIO_PIN_8, 0x30, &hi2c1, "Front_Sensor");

// Initialize and set mode in one call
if (sensor.Init(VL53_Accuracy::FAST)) {
    // Sensor initialized with FAST mode
}

// Or change mode after initialization
if (sensor.SetMeasurementMode(VL53_Accuracy::HIGH)) {
    // Mode changed to HIGH accuracy with optimized timing
}
```

### 2. Application-Specific Mode Selection

#### Obstacle Detection System
```cpp
// Fast response needed for real-time navigation
sensor_front.SetMeasurementMode(VL53_Accuracy::FAST);
sensor_back.SetMeasurementMode(VL53_Accuracy::FAST);

// Check for obstacles
if (sensor_front < 200) {  // Using operator overload
    // Obstacle detected - take evasive action
}
```

#### Precise Lifter Positioning
```cpp
// High accuracy needed for precise positioning
lifter_sensor.SetMeasurementMode(VL53_Accuracy::HIGH);

// Get precise position
uint16_t position = lifter_sensor.GetDistance();
if (lifter_sensor.IsLastMeasurementValid()) {
    // Use precise position for control
    adjustLifterPosition(position);
}
```

#### High-Frequency Scanning
```cpp
// Ultra-fast mode for rapid scanning
scanner_sensor.SetMeasurementMode(VL53_Accuracy::ULTRA_FAST);

// Rapid measurement loop
for (int i = 0; i < 100; i++) {
    uint16_t distance = scanner_sensor.GetDistance();
    // Process measurement quickly
    processQuickMeasurement(distance);
}
```

#### Long-Range Monitoring
```cpp
// Extended range for perimeter detection
perimeter_sensor.SetMeasurementMode(VL53_Accuracy::LONG_RANGE);

// Monitor large area
uint16_t range = perimeter_sensor.GetDistance();
if (range > 2000) {
    // Extended range detection
}
```

### 3. Manual Accuracy Setting (Without Timing Change)

```cpp
// Set accuracy mode only (keeps current timing budget)
sensor.SetAccuracy(VL53_Accuracy::BETTER);

// Check current mode
VL53_Accuracy current = sensor.GetCurrentAccuracy();
if (current == VL53_Accuracy::BETTER) {
    // Mode confirmed
}
```

### 4. Dynamic Mode Switching

```cpp
void adaptiveModeSelection() {
    // Start with balanced mode
    sensor.SetMeasurementMode(VL53_Accuracy::BETTER);
    
    // Monitor performance and adapt
    for (int i = 0; i < 100; i++) {
        uint32_t start = HAL_GetTick();
        uint16_t distance = sensor.GetDistance();
        uint32_t measurement_time = HAL_GetTick() - start;
        
        // Switch to faster mode if we need quicker response
        if (measurement_time > 50 && distance < 500) {
            sensor.SetMeasurementMode(VL53_Accuracy::FAST);
            printf("Switched to FAST mode for close obstacles\\n");
        }
        // Switch to high accuracy for precise measurements
        else if (distance > 1000) {
            sensor.SetMeasurementMode(VL53_Accuracy::HIGH);
            printf("Switched to HIGH mode for distant objects\\n");
        }
        
        HAL_Delay(100);
    }
}
```

### 5. Multi-Sensor with Different Modes

```cpp
// Different sensors optimized for different tasks
VL53L0X_Enhanced obstacle_sensor(GPIOA, GPIO_PIN_8, 0x30, &hi2c1, "Obstacle");
VL53L0X_Enhanced position_sensor(GPIOA, GPIO_PIN_9, 0x31, &hi2c1, "Position");
VL53L0X_Enhanced scan_sensor(GPIOA, GPIO_PIN_10, 0x32, &hi2c1, "Scanner");

void setupMultiSensorSystem() {
    // Fast obstacle detection
    obstacle_sensor.Init();
    obstacle_sensor.SetMeasurementMode(VL53_Accuracy::FAST);
    
    // Precise positioning
    position_sensor.Init();
    position_sensor.SetMeasurementMode(VL53_Accuracy::HIGH);
    
    // Ultra-fast scanning
    scan_sensor.Init();
    scan_sensor.SetMeasurementMode(VL53_Accuracy::ULTRA_FAST);
}

void runMultiSensorTask() {
    // Each sensor optimized for its purpose
    uint16_t obstacle_dist = obstacle_sensor.GetDistance();  // ~20ms
    uint16_t precise_pos = position_sensor.GetDistance();    // ~200ms
    uint16_t scan_result = scan_sensor.GetDistance();        // ~13ms
    
    // Process results based on application needs
}
```

## Mode Selection Guidelines

### Choose ULTRA_FAST when:
- ✅ Maximum speed is critical
- ✅ Doing rapid scanning or motion detection
- ✅ Can tolerate ±5% accuracy
- ✅ Need measurements faster than 15ms

### Choose FAST when:
- ✅ Real-time obstacle avoidance
- ✅ Navigation systems
- ✅ Need good balance of speed and accuracy
- ✅ ±3% accuracy is sufficient

### Choose BETTER when:
- ✅ General purpose applications
- ✅ Not sure which mode to use (good default)
- ✅ Need balance of all factors
- ✅ ±1% accuracy is adequate

### Choose HIGH when:
- ✅ Precise positioning required
- ✅ Measurement verification
- ✅ Can accept slower response (~200ms)
- ✅ Need ±0.5% accuracy

### Choose LONG_RANGE when:
- ✅ Need maximum detection range
- ✅ Large area monitoring
- ✅ Perimeter detection
- ✅ Can accept slowest response (~300ms)

## Technical Details

### Timing Budgets by Mode
- **ULTRA_FAST**: 13,000 μs (13ms)
- **FAST**: 20,000 μs (20ms)
- **BETTER**: 33,000 μs (33ms)
- **HIGH**: 200,000 μs (200ms)
- **LONG_RANGE**: 300,000 μs (300ms)

### VCSEL Pulse Periods by Mode
- **ULTRA_FAST**: Pre=10, Final=6
- **FAST**: Pre=12, Final=8
- **BETTER**: Pre=14, Final=10
- **HIGH**: Pre=16, Final=12
- **LONG_RANGE**: Pre=18, Final=14

## Best Practices

1. **Initialize with appropriate mode**:
   ```cpp
   sensor.Init(VL53_Accuracy::FAST);  // Set mode during init
   ```

2. **Use SetMeasurementMode() for complete configuration**:
   ```cpp
   sensor.SetMeasurementMode(VL53_Accuracy::HIGH);  // Sets timing + accuracy
   ```

3. **Use SetAccuracy() for fine-tuning**:
   ```cpp
   sensor.SetAccuracy(VL53_Accuracy::BETTER);  // Accuracy only
   ```

4. **Monitor performance**:
   ```cpp
   VL53_Accuracy current = sensor.GetCurrentAccuracy();
   ```

5. **Test different modes for your application**:
   - Start with BETTER mode
   - Switch to FAST if speed is critical
   - Switch to HIGH if accuracy is critical
   - Consider ULTRA_FAST for scanning applications
   - Use LONG_RANGE for extended detection

Remember: The choice of accuracy mode depends on your specific application requirements. Test different modes to find the best balance for your use case!
