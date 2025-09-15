/**
 * @file VL53L0X_Class.h
 * @brief Improved VL53L0X Time-of-Flight Sensor C++ Class
 * 
 * Enhanced C++ class for VL53L0X sensor with individual initialization,
 * calibration, and ranging capabilities. Each sensor instance is completely
 * independent and can be used separately.
 * 
 * Features:
 * - Individual sensor initialization with unique I2C addresses
 * - Independent calibration for each sensor
 * - Flexible measurement modes (single-shot, continuous)
 * - Comprehensive error handling and status reporting
 * - Easy integration with multiple sensors
 * 
 * @author Based on existing VL53.h design
 * @date 2025
 */

#ifndef VL53L0X_CLASS_H_
#define VL53L0X_CLASS_H_

#include "stm32h7xx_hal.h"
#include "../Drivers/VL53L0X/vl53l0x_api.h"
#include <stdint.h>
#include <string>

/**
 * @brief VL53L0X sensor measurement status enumeration
 */
enum class VL53_Status {
    VALID = 0,           /*!< Measurement is valid */
    OUT_OF_RANGE,        /*!< Target out of measurement range */
    SIGMA_TOO_HIGH,      /*!< Measurement uncertainty too high */
    SIGNAL_TOO_LOW,      /*!< Return signal too weak */
    PHASE_ERROR,         /*!< Phase measurement error */
    HARDWARE_ERROR,      /*!< Hardware/communication error */
    NOT_INITIALIZED,     /*!< Sensor not initialized */
    TIMEOUT              /*!< Measurement timeout */
};

/**
 * @brief VL53L0X sensor accuracy modes
 * 
 * Different accuracy modes provide trade-offs between measurement speed and precision:
 * - FAST: ~20ms measurement time, ±3% accuracy, good for obstacle detection
 * - BETTER: ~33ms measurement time, ±1% accuracy, balanced performance  
 * - HIGH: ~200ms measurement time, ±0.5% accuracy, best for precise positioning
 * - ULTRA_FAST: ~13ms measurement time, ±5% accuracy, maximum speed
 * - LONG_RANGE: ~300ms measurement time, ±1% accuracy, extended range capability
 */
enum class VL53_Accuracy {
    ULTRA_FAST = 0,      /*!< Ultra fast measurement, lowest accuracy (~13ms) */
    FAST,                /*!< Fast measurement, lower accuracy (~20ms) */
    BETTER,              /*!< Balanced speed and accuracy (~33ms) */
    HIGH,                /*!< High accuracy, slower measurement (~200ms) */
    LONG_RANGE           /*!< Extended range capability, slowest (~300ms) */
};

/**
 * @brief VL53L0X measurement result structure
 */
struct VL53_MeasurementData {
    uint16_t distance_mm;        /*!< Distance in millimeters */
    VL53_Status status;          /*!< Measurement status */
    uint16_t signal_rate;        /*!< Signal rate (MCPS) */
    uint16_t ambient_rate;       /*!< Ambient rate (MCPS) */
    uint32_t measurement_time;   /*!< Measurement timestamp */
    bool is_valid;               /*!< Quick validity check */
    
    VL53_MeasurementData() : distance_mm(0), status(VL53_Status::NOT_INITIALIZED), 
                            signal_rate(0), ambient_rate(0), measurement_time(0), is_valid(false) {}
};

/**
 * @brief Enhanced VL53L0X sensor class
 */
class VL53L0X_Enhanced {
private:
    // Hardware configuration
    VL53L0X_Dev_t vl53l0x_device;
    VL53L0X_Dev_t *vl53_dev;
    GPIO_TypeDef *xshut_port;
    uint16_t xshut_pin;
    uint8_t i2c_address;
    I2C_HandleTypeDef *hi2c;
    
    // Measurement data
    VL53L0X_RangingMeasurementData_t ranging_data;
    VL53_MeasurementData last_measurement;
    
    // Status and configuration
    bool is_initialized;
    bool is_calibrated;
    bool is_continuous_mode;
    VL53_Accuracy accuracy_mode;
    uint32_t timing_budget_us;
    uint32_t error_count;
    std::string sensor_id;
    
    // Private methods
    VL53_Status ConvertDeviceStatus(uint8_t device_status);
    VL53L0X_Error InitializeDevice();
    VL53L0X_Error ApplyConfiguration();
    void ResetSensor();
    void UpdateLastMeasurement();

public:
    /**
     * @brief Constructor
     * @param port GPIO port for XSHUT pin
     * @param pin GPIO pin for XSHUT
     * @param address Unique I2C address (7-bit format)
     * @param hi2c_handle Pointer to I2C handle
     * @param id Optional sensor identifier string
     */
    VL53L0X_Enhanced(GPIO_TypeDef *port, uint16_t pin, uint8_t address, 
                     I2C_HandleTypeDef *hi2c_handle, const char* id = nullptr);
    
    /**
     * @brief Destructor
     */
    ~VL53L0X_Enhanced();
    
    // Initialization methods
    
    /**
     * @brief Initialize sensor with default settings
     * @return true if successful, false otherwise
     */
    bool Init();
    
    /**
     * @brief Initialize sensor with custom accuracy mode
     * @param accuracy Desired accuracy mode
     * @param timing_budget_us Measurement timing budget in microseconds
     * @return true if successful, false otherwise
     */
    bool Init(VL53_Accuracy accuracy, uint32_t timing_budget_us = 26000);
    
    /**
     * @brief Check if sensor is properly initialized
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const { return is_initialized; }
    
    // Measurement methods
    
    /**
     * @brief Perform single distance measurement
     * @return Distance in millimeters (0 if invalid)
     */
    uint16_t GetDistance();
    
    /**
     * @brief Perform single measurement with status
     * @param distance Reference to store distance result
     * @param status Reference to store measurement status
     * @return true if measurement completed (check status for validity)
     */
    bool GetDistance(uint16_t &distance, VL53_Status &status);
    
    /**
     * @brief Get detailed measurement data
     * @param data Reference to store measurement data
     * @return true if measurement completed
     */
    bool GetMeasurement(VL53_MeasurementData &data);
    
    /**
     * @brief Update sensor reading (call regularly in main loop)
     */
    void Update();
    
    /**
     * @brief Get last measurement result
     * @return Last valid distance in millimeters
     */
    uint16_t GetLastDistance() const { return last_measurement.distance_mm; }
    
    /**
     * @brief Get last measurement status
     * @return Last measurement status
     */
    VL53_Status GetLastStatus() const { return last_measurement.status; }
    
    /**
     * @brief Check if last measurement was valid
     * @return true if last measurement was valid
     */
    bool IsLastMeasurementValid() const { return last_measurement.is_valid; }
    
    // Continuous measurement methods
    
    /**
     * @brief Start continuous measurement mode
     * @param period_ms Measurement period in milliseconds
     * @return true if successful
     */
    bool StartContinuous(uint32_t period_ms = 100);
    
    /**
     * @brief Stop continuous measurement mode
     * @return true if successful
     */
    bool StopContinuous();
    
    /**
     * @brief Check if new data is available (for continuous mode)
     * @return true if new data is ready
     */
    bool IsDataReady();
    
    /**
     * @brief Check if sensor is in continuous mode
     * @return true if in continuous mode
     */
    bool IsContinuousMode() const { return is_continuous_mode; }
    
    // Calibration methods
    
    /**
     * @brief Perform offset calibration
     * @param target_distance_mm Known target distance in millimeters
     * @return true if calibration successful
     */
    bool CalibrateOffset(uint16_t target_distance_mm = 100);
    
    /**
     * @brief Perform crosstalk calibration (no target should be present)
     * @return true if calibration successful
     */
    bool CalibrateCrosstalk();
    
    /**
     * @brief Perform complete calibration sequence
     * @param target_distance_mm Known target distance for offset calibration
     * @return true if all calibrations successful
     */
    bool PerformFullCalibration(uint16_t target_distance_mm = 100);
    
    /**
     * @brief Check if sensor is calibrated
     * @return true if calibrated
     */
    bool IsCalibrated() const { return is_calibrated; }
    
    // Configuration methods
    
    /**
     * @brief Set measurement accuracy mode
     * @param accuracy New accuracy mode
     * @return true if successful
     */
    bool SetAccuracy(VL53_Accuracy accuracy);
    
    /**
     * @brief Set measurement mode with predefined configurations
     * @param mode Accuracy mode with optimized settings
     * @return true if successful, false if error
     * 
     * Predefined modes:
     * - ULTRA_FAST: 13ms timing, optimized for speed
     * - FAST: 20ms timing, good for real-time applications
     * - BETTER: 33ms timing, balanced performance
     * - HIGH: 200ms timing, maximum accuracy for positioning
     * - LONG_RANGE: 300ms timing, extended detection range
     */
    bool SetMeasurementMode(VL53_Accuracy mode);
    
    /**
     * @brief Get current accuracy mode
     * @return Current accuracy mode
     */
    VL53_Accuracy GetCurrentAccuracy() const { return accuracy_mode; }
    
    /**
     * @brief Set measurement timing budget
     * @param timing_budget_us Timing budget in microseconds
     * @return true if successful
     */
    bool SetTimingBudget(uint32_t timing_budget_us);
    
    /**
     * @brief Set signal rate threshold
     * @param threshold Signal rate threshold (MCPS * 65536)
     * @return true if successful
     */
    bool SetSignalThreshold(uint32_t threshold);
    
    /**
     * @brief Change sensor I2C address
     * @param new_address New I2C address (7-bit format)
     * @return true if successful
     */
    bool SetI2CAddress(uint8_t new_address);
    
    // Status and information methods
    
    /**
     * @brief Get sensor information string
     * @return Sensor information as string
     */
    std::string GetSensorInfo() const;
    
    /**
     * @brief Get current I2C address
     * @return Current I2C address (7-bit format)
     */
    uint8_t GetI2CAddress() const { return i2c_address; }
    
    /**
     * @brief Get sensor ID string
     * @return Sensor identifier string
     */
    std::string GetSensorID() const { return sensor_id; }
    
    /**
     * @brief Get error count
     * @return Number of errors encountered
     */
    uint32_t GetErrorCount() const { return error_count; }
    
    /**
     * @brief Reset error count
     */
    void ResetErrorCount() { error_count = 0; }
    
    /**
     * @brief Reset sensor to default state
     * @return true if successful
     */
    bool Reset();
    
    /**
     * @brief Get status string for debugging
     * @param status Status to convert
     * @return Status as string
     */
    static std::string StatusToString(VL53_Status status);
    
    // Operator overloads for easy usage
    
    /**
     * @brief Implicit conversion to distance value
     * @return Current distance measurement
     */
    operator uint16_t() const { return GetLastDistance(); }
    
    /**
     * @brief Assignment operator for easy comparison
     * @param distance Distance to compare
     * @return Reference to this object
     */
    bool operator>(uint16_t distance) const { return GetLastDistance() > distance; }
    bool operator<(uint16_t distance) const { return GetLastDistance() < distance; }
    bool operator>=(uint16_t distance) const { return GetLastDistance() >= distance; }
    bool operator<=(uint16_t distance) const { return GetLastDistance() <= distance; }
};

/**
 * @brief Multi-sensor manager class for coordinated operations
 */
class VL53L0X_MultiSensor {
private:
    VL53L0X_Enhanced** sensors;
    uint8_t sensor_count;
    bool is_synchronized;
    
public:
    /**
     * @brief Constructor
     * @param sensor_array Array of VL53L0X_Enhanced pointers
     * @param count Number of sensors
     */
    VL53L0X_MultiSensor(VL53L0X_Enhanced** sensor_array, uint8_t count);
    
    /**
     * @brief Initialize all sensors
     * @return true if all sensors initialized successfully
     */
    bool InitAll();
    
    /**
     * @brief Update all sensors
     */
    void UpdateAll();
    
    /**
     * @brief Get distances from all sensors
     * @param distances Array to store distance results
     * @param statuses Array to store status results (optional)
     * @return Number of valid measurements
     */
    uint8_t GetAllDistances(uint16_t* distances, VL53_Status* statuses = nullptr);
    
    /**
     * @brief Start continuous mode on all sensors
     * @param period_ms Measurement period
     * @return true if all sensors started successfully
     */
    bool StartAllContinuous(uint32_t period_ms = 100);
    
    /**
     * @brief Stop continuous mode on all sensors
     * @return true if all sensors stopped successfully
     */
    bool StopAllContinuous();
    
    /**
     * @brief Calibrate all sensors
     * @param target_distance_mm Target distance for calibration
     * @return true if all sensors calibrated successfully
     */
    bool CalibrateAll(uint16_t target_distance_mm = 100);
    
    /**
     * @brief Get status summary of all sensors
     * @return Status string for all sensors
     */
    std::string GetStatusSummary() const;
};

#endif /* VL53L0X_CLASS_H_ */
