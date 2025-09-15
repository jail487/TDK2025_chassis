/**
 * @file VL53L0X_Class_Example.cpp
 * @brief Enhanced VL53L0X C++ Class Usage Examples
 * 
 * This file demonstrates how to use the enhanced VL53L0X C++ class
 * for individual sensor initialization, calibration, and measurement.
 * 
 * @author Based on existing design patterns
 * @date 2025
 */

#include "VL53L0X_Class.h"
#include "main.h"
#include <cstdio>

// External I2C handle
extern I2C_HandleTypeDef hi2c1;

// Global sensor instances - each sensor is independent
VL53L0X_Enhanced sensor_front(GPIOA, GPIO_PIN_8, 0x30, &hi2c1, "Front_Sensor");
VL53L0X_Enhanced sensor_right(GPIOA, GPIO_PIN_9, 0x31, &hi2c1, "Right_Sensor");
VL53L0X_Enhanced sensor_back(GPIOA, GPIO_PIN_10, 0x32, &hi2c1, "Back_Sensor");
VL53L0X_Enhanced sensor_left(GPIOA, GPIO_PIN_11, 0x33, &hi2c1, "Left_Sensor");

// Additional sensors for other applications
VL53L0X_Enhanced sensor_lifter_top(GPIOB, GPIO_PIN_12, 0x34, &hi2c1, "Lifter_Top");
VL53L0X_Enhanced sensor_lifter_bottom(GPIOB, GPIO_PIN_13, 0x35, &hi2c1, "Lifter_Bottom");

// Multi-sensor manager for coordinated operations
VL53L0X_Enhanced* chassis_sensors[] = {&sensor_front, &sensor_right, &sensor_back, &sensor_left};
VL53L0X_MultiSensor chassis_manager(chassis_sensors, 4);

/**
 * @brief Example 1: Individual sensor initialization and basic usage
 */
void Example_IndividualSensorUsage()
{
    printf("=== Example 1: Individual Sensor Usage ===\\n");
    
    // Initialize front sensor only
    if (sensor_front.Init()) {
        printf("Front sensor initialized successfully\\n");
        
        // Perform some measurements
        for (int i = 0; i < 5; i++) {
            uint16_t distance;
            VL53_Status status;
            
            if (sensor_front.GetDistance(distance, status)) {
                printf("Measurement %d: %d mm [%s]\\n", 
                       i + 1, distance, 
                       VL53L0X_Enhanced::StatusToString(status).c_str());
            }
            
            HAL_Delay(200);
        }
        
        // Show sensor information
        printf("\\n%s\\n", sensor_front.GetSensorInfo().c_str());
    } else {
        printf("Front sensor initialization failed\\n");
    }
}

/**
 * @brief Example 2: Multiple independent sensors
 */
void Example_MultipleIndependentSensors()
{
    printf("\\n=== Example 2: Multiple Independent Sensors ===\\n");
    
    // Initialize each sensor individually
    bool front_ok = sensor_front.Init(VL53_Accuracy::FAST);
    bool right_ok = sensor_right.Init(VL53_Accuracy::BETTER);
    bool back_ok = sensor_back.Init(VL53_Accuracy::HIGH, 50000); // High accuracy with 50ms budget
    bool left_ok = sensor_left.Init();
    
    printf("Initialization results:\\n");
    printf("Front: %s, Right: %s, Back: %s, Left: %s\\n",
           front_ok ? "OK" : "FAIL",
           right_ok ? "OK" : "FAIL", 
           back_ok ? "OK" : "FAIL",
           left_ok ? "OK" : "FAIL");
    
    // Use sensors independently
    if (front_ok) {
        uint16_t front_distance = sensor_front.GetDistance();
        printf("Front distance: %d mm\\n", front_distance);
    }
    
    if (right_ok) {
        VL53_MeasurementData right_data;
        if (sensor_right.GetMeasurement(right_data)) {
            printf("Right sensor - Distance: %d mm, Signal: %d, Valid: %s\\n",
                   right_data.distance_mm, right_data.signal_rate,
                   right_data.is_valid ? "Yes" : "No");
        }
    }
    
    // Use operator overloads for easy comparison
    if (back_ok && sensor_back > (uint16_t)100) {
        printf("Back sensor detects obstacle closer than 100mm!\\n");
    }
}

/**
 * @brief Example 3: Individual sensor calibration
 */
void Example_IndividualCalibration()
{
    printf("\\n=== Example 3: Individual Sensor Calibration ===\\n");
    
    // Calibrate front sensor only
    if (sensor_front.IsInitialized()) {
        printf("Calibrating front sensor...\\n");
        printf("Please place target at 100mm and press any key\\n");
        HAL_Delay(3000); // Wait for user to position target
        
        // Perform calibration
        bool calibration_ok = sensor_front.PerformFullCalibration(100);
        
        if (calibration_ok) {
            printf("Front sensor calibration completed successfully\\n");
            
            // Test calibration by taking measurements
            printf("Testing calibrated sensor:\\n");
            for (int i = 0; i < 3; i++) {
                uint16_t distance = sensor_front.GetDistance();
                printf("Calibrated measurement %d: %d mm\\n", i + 1, distance);
                HAL_Delay(500);
            }
        } else {
            printf("Front sensor calibration failed\\n");
        }
    }
    
    // Calibrate right sensor with different target distance
    if (sensor_right.IsInitialized()) {
        printf("\\nCalibrating right sensor with 150mm target...\\n");
        HAL_Delay(2000);
        
        if (sensor_right.CalibrateOffset(150)) {
            printf("Right sensor offset calibration successful\\n");
        }
        
        if (sensor_right.CalibrateCrosstalk()) {
            printf("Right sensor crosstalk calibration successful\\n");
        }
    }
}

/**
 * @brief Example 4: Continuous measurement with individual sensors
 */
void Example_ContinuousMeasurement()
{
    printf("\\n=== Example 4: Continuous Measurement ===\\n");
    
    // Start continuous mode on front sensor only
    if (sensor_front.IsInitialized()) {
        printf("Starting continuous measurement on front sensor\\n");
        
        if (sensor_front.StartContinuous(100)) { // 100ms interval
            printf("Continuous mode started. Reading for 5 seconds...\\n");
            
            uint32_t start_time = HAL_GetTick();
            int measurement_count = 0;
            
            while ((HAL_GetTick() - start_time) < 5000) {
                if (sensor_front.IsDataReady()) {
                    VL53_MeasurementData data;
                    if (sensor_front.GetMeasurement(data)) {
                        measurement_count++;
                        printf("Continuous %d: %d mm [%s]\\n",
                               measurement_count, data.distance_mm,
                               VL53L0X_Enhanced::StatusToString(data.status).c_str());
                    }
                }
                HAL_Delay(10);
            }
            
            // Stop continuous mode
            sensor_front.StopContinuous();
            printf("Continuous mode stopped. Total measurements: %d\\n", measurement_count);
        }
    }
}

/**
 * @brief Example 5: Multi-sensor manager usage
 */
void Example_MultiSensorManager()
{
    printf("\\n=== Example 5: Multi-Sensor Manager ===\\n");
    
    // Initialize all chassis sensors at once
    if (chassis_manager.InitAll()) {
        printf("All chassis sensors initialized successfully\\n");
        
        // Get synchronized measurements
        uint16_t distances[4];
        VL53_Status statuses[4];
        const char* sensor_names[] = {"Front", "Right", "Back", "Left"};
        
        uint8_t valid_count = chassis_manager.GetAllDistances(distances, statuses);
        
        printf("Synchronized measurements (%d valid):\\n", valid_count);
        for (int i = 0; i < 4; i++) {
            printf("  %s: %d mm [%s]\\n", 
                   sensor_names[i], distances[i],
                   VL53L0X_Enhanced::StatusToString(statuses[i]).c_str());
        }
        
        // Show status summary
        printf("\\n%s\\n", chassis_manager.GetStatusSummary().c_str());
        
        // Start continuous mode on all sensors
        if (chassis_manager.StartAllContinuous(150)) {
            printf("All sensors in continuous mode\\n");
            
            // Update and read multiple times
            for (int cycle = 0; cycle < 3; cycle++) {
                HAL_Delay(200);
                chassis_manager.UpdateAll();
                
                valid_count = chassis_manager.GetAllDistances(distances, statuses);
                printf("Cycle %d (%d valid): F:%d R:%d B:%d L:%d\\n",
                       cycle + 1, valid_count, 
                       distances[0], distances[1], distances[2], distances[3]);
            }
            
            chassis_manager.StopAllContinuous();
        }
    } else {
        printf("Multi-sensor initialization failed\\n");
    }
}

/**
 * @brief Example 6: Specialized sensor applications
 */
void Example_SpecializedApplications()
{
    printf("\\n=== Example 6: Specialized Applications ===\\n");
    
    // Lifter sensors for vertical positioning
    printf("Initializing lifter sensors...\\n");
    
    bool top_ok = sensor_lifter_top.Init(VL53_Accuracy::HIGH);
    bool bottom_ok = sensor_lifter_bottom.Init(VL53_Accuracy::HIGH);
    
    if (top_ok && bottom_ok) {
        printf("Lifter sensors initialized successfully\\n");
        
        // Monitor lifter position
        for (int i = 0; i < 5; i++) {
            uint16_t top_distance = sensor_lifter_top.GetDistance();
            uint16_t bottom_distance = sensor_lifter_bottom.GetDistance();
            
            printf("Lifter position %d - Top: %d mm, Bottom: %d mm\\n",
                   i + 1, top_distance, bottom_distance);
            
            // Calculate lifter height (example logic)
            if (sensor_lifter_top.IsLastMeasurementValid() && 
                sensor_lifter_bottom.IsLastMeasurementValid()) {
                
                uint16_t lifter_height = bottom_distance - top_distance;
                printf("  Calculated lifter height: %d mm\\n", lifter_height);
            }
            
            HAL_Delay(300);
        }
    }
}

/**
 * @brief Example 7: Dynamic sensor configuration
 */
void Example_DynamicConfiguration()
{
    printf("\\n=== Example 7: Dynamic Configuration ===\\n");
    
    if (sensor_front.IsInitialized()) {
        printf("Testing different accuracy modes on front sensor:\\n");
        
        // Test Fast mode
        sensor_front.SetAccuracy(VL53_Accuracy::FAST);
        sensor_front.SetTimingBudget(20000); // 20ms
        uint32_t start_time = HAL_GetTick();
        uint16_t fast_distance = sensor_front.GetDistance();
        uint32_t fast_time = HAL_GetTick() - start_time;
        printf("Fast mode: %d mm in %lu ms\\n", fast_distance, fast_time);
        
        // Test High accuracy mode
        sensor_front.SetAccuracy(VL53_Accuracy::HIGH);
        sensor_front.SetTimingBudget(200000); // 200ms
        start_time = HAL_GetTick();
        uint16_t high_distance = sensor_front.GetDistance();
        uint32_t high_time = HAL_GetTick() - start_time;
        printf("High mode: %d mm in %lu ms\\n", high_distance, high_time);
        
        // Reset to balanced mode
        sensor_front.SetAccuracy(VL53_Accuracy::BETTER);
        sensor_front.SetTimingBudget(26000);
    }
}

/**
 * @brief Main chassis task using individual sensors
 */
void VL53L0X_Chassis_Task_Enhanced()
{
    static bool sensors_initialized = false;
    static uint32_t last_measurement_time = 0;
    
    // Initialize sensors once
    if (!sensors_initialized) {
        printf("Initializing enhanced chassis sensors...\\n");
        
        bool front_ok = sensor_front.Init();
        bool right_ok = sensor_right.Init();
        bool back_ok = sensor_back.Init();
        bool left_ok = sensor_left.Init();
        
        if (front_ok && right_ok && back_ok && left_ok) {
            sensors_initialized = true;
            printf("All chassis sensors initialized successfully\\n");
            
            // Optional: Calibrate all sensors
            // chassis_manager.CalibrateAll(100);
        } else {
            printf("Some chassis sensors failed to initialize\\n");
        }
    }
    
    // Perform measurements every 100ms
    if (sensors_initialized && (HAL_GetTick() - last_measurement_time >= 100)) {
        // Update all sensors
        sensor_front.Update();
        sensor_right.Update();
        sensor_back.Update();
        sensor_left.Update();
        
        // Get distances
        uint16_t front_dist = sensor_front.GetLastDistance();
        uint16_t right_dist = sensor_right.GetLastDistance();
        uint16_t back_dist = sensor_back.GetLastDistance();
        uint16_t left_dist = sensor_left.GetLastDistance();
        
        // Obstacle detection logic
        const uint16_t obstacle_threshold = 200; // 20cm
        
        bool front_obstacle = sensor_front.IsLastMeasurementValid() && front_dist < obstacle_threshold;
        bool right_obstacle = sensor_right.IsLastMeasurementValid() && right_dist < obstacle_threshold;
        bool back_obstacle = sensor_back.IsLastMeasurementValid() && back_dist < obstacle_threshold;
        bool left_obstacle = sensor_left.IsLastMeasurementValid() && left_dist < obstacle_threshold;
        
        // Send to chassis control system
        // chassis_controller_set_obstacles(front_obstacle, right_obstacle, back_obstacle, left_obstacle);
        
        // Debug output (remove in production)
        if (front_obstacle || right_obstacle || back_obstacle || left_obstacle) {
            printf("OBSTACLES DETECTED - F:%s R:%s B:%s L:%s\\n",
                   front_obstacle ? "YES" : "no",
                   right_obstacle ? "YES" : "no", 
                   back_obstacle ? "YES" : "no",
                   left_obstacle ? "YES" : "no");
        }
        
        last_measurement_time = HAL_GetTick();
    }
}

/**
 * @brief Example 8: Accuracy Mode Selection
 * Demonstrates how to select different measurement modes for different applications
 */
void Example_AccuracyModeSelection()
{
    printf("\\n=== Example 8: Accuracy Mode Selection ===\\n");
    
    if (!sensor_front.IsInitialized()) {
        printf("Initializing front sensor for mode testing...\\n");
        if (!sensor_front.Init()) {
            printf("Failed to initialize sensor!\\n");
            return;
        }
    }
    
    // Test all accuracy modes
    VL53_Accuracy modes[] = {
        VL53_Accuracy::ULTRA_FAST,
        VL53_Accuracy::FAST,
        VL53_Accuracy::BETTER,
        VL53_Accuracy::HIGH,
        VL53_Accuracy::LONG_RANGE
    };
    
    const char* mode_names[] = {
        "ULTRA_FAST (13ms)",
        "FAST (20ms)",
        "BETTER (33ms)", 
        "HIGH (200ms)",
        "LONG_RANGE (300ms)"
    };
    
    for (int i = 0; i < 5; i++) {
        printf("\\nTesting %s mode:\\n", mode_names[i]);
        
        // Set measurement mode with predefined timing
        if (sensor_front.SetMeasurementMode(modes[i])) {
            printf("  Mode set successfully\\n");
            
            // Take a few measurements to show timing
            uint32_t start_time = HAL_GetTick();
            
            for (int j = 0; j < 3; j++) {
                uint32_t measure_start = HAL_GetTick();
                uint16_t distance = sensor_front.GetDistance();
                uint32_t measure_time = HAL_GetTick() - measure_start;
                
                if (sensor_front.IsLastMeasurementValid()) {
                    printf("  Measurement %d: %d mm (took %lu ms)\\n", 
                           j+1, distance, measure_time);
                } else {
                    printf("  Measurement %d: Invalid (took %lu ms)\\n", 
                           j+1, measure_time);
                }
                
                HAL_Delay(100); // Small delay between measurements
            }
            
            uint32_t total_time = HAL_GetTick() - start_time;
            printf("  Total time for 3 measurements: %lu ms\\n", total_time);
        } else {
            printf("  Failed to set mode!\\n");
        }
    }
    
    // Demonstrate manual mode setting
    printf("\\nManual accuracy setting (without timing budget change):\\n");
    if (sensor_front.SetAccuracy(VL53_Accuracy::BETTER)) {
        printf("  Accuracy set to BETTER mode\\n");
        printf("  Current mode: %d\\n", (int)sensor_front.GetCurrentAccuracy());
    }
}

/**
 * @brief Example 9: Application-Specific Mode Selection
 * Shows how to choose the right mode for different use cases
 */
void Example_ApplicationSpecificModes()
{
    printf("\\n=== Example 9: Application-Specific Mode Selection ===\\n");
    
    // Obstacle detection mode (fast response needed)
    printf("\\n1. Obstacle Detection Setup:\\n");
    if (sensor_front.SetMeasurementMode(VL53_Accuracy::FAST)) {
        printf("  Using FAST mode for obstacle detection\\n");
        printf("  Good for: Real-time navigation, collision avoidance\\n");
        printf("  Characteristics: ~20ms response, ±3%% accuracy\\n");
    }
    
    // Precise positioning mode (accuracy important)
    printf("\\n2. Precise Positioning Setup:\\n");
    if (sensor_lifter_top.Init() && sensor_lifter_top.SetMeasurementMode(VL53_Accuracy::HIGH)) {
        printf("  Using HIGH mode for lifter positioning\\n");
        printf("  Good for: Precise positioning, measurement verification\\n");
        printf("  Characteristics: ~200ms response, ±0.5%% accuracy\\n");
    }
    
    // Long range detection mode
    printf("\\n3. Long Range Detection Setup:\\n");
    if (sensor_back.Init() && sensor_back.SetMeasurementMode(VL53_Accuracy::LONG_RANGE)) {
        printf("  Using LONG_RANGE mode for extended detection\\n");
        printf("  Good for: Large area monitoring, perimeter detection\\n");
        printf("  Characteristics: ~300ms response, extended range\\n");
    }
    
    // Ultra-fast mode for high-frequency sampling
    printf("\\n4. High-Frequency Sampling Setup:\\n");
    if (sensor_right.Init() && sensor_right.SetMeasurementMode(VL53_Accuracy::ULTRA_FAST)) {
        printf("  Using ULTRA_FAST mode for high-frequency sampling\\n");
        printf("  Good for: Rapid scanning, motion detection\\n");
        printf("  Characteristics: ~13ms response, maximum speed\\n");
    }
    
    printf("\\nMode selection guidelines:\\n");
    printf("  - ULTRA_FAST: Motion detection, rapid scanning\\n");
    printf("  - FAST: Obstacle avoidance, real-time navigation\\n");
    printf("  - BETTER: General purpose, balanced performance\\n");
    printf("  - HIGH: Precise positioning, measurement verification\\n");
    printf("  - LONG_RANGE: Extended detection, large area monitoring\\n");
}

/**
 * @brief Run all examples
 */
void VL53L0X_Run_All_Examples()
{
    printf("VL53L0X Enhanced C++ Class Examples\\n");
    printf("===================================\\n");
    
    Example_IndividualSensorUsage();
    HAL_Delay(1000);
    
    Example_MultipleIndependentSensors();
    HAL_Delay(1000);
    
    Example_IndividualCalibration();
    HAL_Delay(1000);
    
    Example_ContinuousMeasurement();
    HAL_Delay(1000);
    
    Example_MultiSensorManager();
    HAL_Delay(1000);
    
    Example_SpecializedApplications();
    HAL_Delay(1000);
    
    Example_DynamicConfiguration();
    HAL_Delay(1000);
    
    Example_AccuracyModeSelection();
    HAL_Delay(1000);
    
    Example_ApplicationSpecificModes();
    
    printf("\\nAll examples completed!\\n");
    printf("You can now use VL53L0X_Chassis_Task_Enhanced() in your main loop\\n");
}
