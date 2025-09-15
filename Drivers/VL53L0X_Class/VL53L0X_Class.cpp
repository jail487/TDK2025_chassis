/**
 * @file VL53L0X_Class.cpp
 * @brief Enhanced VL53L0X Time-of-Flight Sensor C++ Class Implementation
 * 
 * @author Based on existing VL53.cpp design
 * @date 2025
 */

#include "VL53L0X_Class.h"
#include <cstring>
#include <cstdio>

extern I2C_HandleTypeDef hi2c1; // Default I2C handle

// Constructor
VL53L0X_Enhanced::VL53L0X_Enhanced(GPIO_TypeDef *port, uint16_t pin, uint8_t address, 
                                   I2C_HandleTypeDef *hi2c_handle, const char* id)
    : xshut_port(port), xshut_pin(pin), i2c_address(address), hi2c(hi2c_handle),
      is_initialized(false), is_calibrated(false), is_continuous_mode(false),
      accuracy_mode(VL53_Accuracy::BETTER), timing_budget_us(26000), error_count(0)
{
    // Initialize device structure
    memset(&vl53l0x_device, 0, sizeof(VL53L0X_Dev_t));
    memset(&ranging_data, 0, sizeof(VL53L0X_RangingMeasurementData_t));
    
    vl53_dev = &vl53l0x_device;
    
    // Set sensor ID
    if (id != nullptr) {
        sensor_id = std::string(id);
    } else {
        char temp_id[32];
        snprintf(temp_id, sizeof(temp_id), "VL53L0X_0x%02X", address);
        sensor_id = std::string(temp_id);
    }
    
    // Use provided I2C handle or default
    if (hi2c == nullptr) {
        hi2c = &hi2c1;
    }
}

// Destructor
VL53L0X_Enhanced::~VL53L0X_Enhanced()
{
    if (is_continuous_mode) {
        StopContinuous();
    }
}

// Initialize sensor with default settings
bool VL53L0X_Enhanced::Init()
{
    return Init(VL53_Accuracy::BETTER, 26000);
}

// Initialize sensor with custom settings
bool VL53L0X_Enhanced::Init(VL53_Accuracy accuracy, uint32_t timing_budget_us)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    
    // Reset sensor
    ResetSensor();
    
    // Store configuration
    accuracy_mode = accuracy;
    this->timing_budget_us = timing_budget_us;
    
    // Setup device structure
    vl53_dev->I2cHandle = hi2c;
    vl53_dev->I2cDevAddr = 0x52; // Default address (0x29 << 1)
    
    // Enable sensor
    HAL_GPIO_WritePin(xshut_port, xshut_pin, GPIO_PIN_SET);
    HAL_Delay(100);
    
    // Set unique I2C address if different from default
    if (i2c_address != 0x29) {
        status = VL53L0X_SetDeviceAddress(vl53_dev, i2c_address << 1);
        if (status != VL53L0X_ERROR_NONE) {
            error_count++;
            printf("%s: Failed to set I2C address 0x%02X\\n", sensor_id.c_str(), i2c_address);
            return false;
        }
        vl53_dev->I2cDevAddr = i2c_address << 1;
    }
    
    // Initialize device
    status = InitializeDevice();
    if (status != VL53L0X_ERROR_NONE) {
        error_count++;
        printf("%s: Device initialization failed\\n", sensor_id.c_str());
        return false;
    }
    
    // Apply configuration
    status = ApplyConfiguration();
    if (status != VL53L0X_ERROR_NONE) {
        error_count++;
        printf("%s: Configuration failed\\n", sensor_id.c_str());
        return false;
    }
    
    is_initialized = true;
    printf("%s: Initialization successful\\n", sensor_id.c_str());
    return true;
}

// Perform single distance measurement
uint16_t VL53L0X_Enhanced::GetDistance()
{
    uint16_t distance;
    VL53_Status status;
    
    if (GetDistance(distance, status)) {
        return (status == VL53_Status::VALID) ? distance : 0;
    }
    
    return 0;
}

// Perform single measurement with status
bool VL53L0X_Enhanced::GetDistance(uint16_t &distance, VL53_Status &status)
{
    if (!is_initialized) {
        distance = 0;
        status = VL53_Status::NOT_INITIALIZED;
        return false;
    }
    
    VL53L0X_Error vl53_status = VL53L0X_PerformSingleRangingMeasurement(vl53_dev, &ranging_data);
    
    if (vl53_status == VL53L0X_ERROR_NONE) {
        distance = ranging_data.RangeMilliMeter;
        status = ConvertDeviceStatus(ranging_data.RangeStatus);
        
        // Update last measurement
        last_measurement.distance_mm = distance;
        last_measurement.status = status;
        last_measurement.signal_rate = (uint16_t)(ranging_data.SignalRateRtnMegaCps >> 16);
        last_measurement.ambient_rate = (uint16_t)(ranging_data.AmbientRateRtnMegaCps >> 16);
        last_measurement.measurement_time = HAL_GetTick();
        last_measurement.is_valid = (status == VL53_Status::VALID);
        
        return true;
    } else {
        error_count++;
        distance = 0;
        status = VL53_Status::HARDWARE_ERROR;
        last_measurement.is_valid = false;
        return false;
    }
}

// Get detailed measurement data
bool VL53L0X_Enhanced::GetMeasurement(VL53_MeasurementData &data)
{
    uint16_t distance;
    VL53_Status status;
    
    bool result = GetDistance(distance, status);
    data = last_measurement;
    
    return result;
}

// Update sensor reading
void VL53L0X_Enhanced::Update()
{
    if (!is_initialized) return;
    
    uint16_t distance;
    VL53_Status status;
    GetDistance(distance, status);
}

// Start continuous measurement mode
bool VL53L0X_Enhanced::StartContinuous(uint32_t period_ms)
{
    if (!is_initialized) {
        return false;
    }
    
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    
    // Set inter-measurement period
    status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(vl53_dev, period_ms);
    if (status != VL53L0X_ERROR_NONE) {
        error_count++;
        return false;
    }
    
    // Start continuous ranging
    status = VL53L0X_StartMeasurement(vl53_dev);
    if (status == VL53L0X_ERROR_NONE) {
        is_continuous_mode = true;
        printf("%s: Continuous mode started (period: %lu ms)\\n", sensor_id.c_str(), period_ms);
        return true;
    }
    
    error_count++;
    return false;
}

// Stop continuous measurement mode
bool VL53L0X_Enhanced::StopContinuous()
{
    if (!is_initialized || !is_continuous_mode) {
        return false;
    }
    
    VL53L0X_Error status = VL53L0X_StopMeasurement(vl53_dev);
    
    if (status == VL53L0X_ERROR_NONE) {
        is_continuous_mode = false;
        printf("%s: Continuous mode stopped\\n", sensor_id.c_str());
        return true;
    }
    
    error_count++;
    return false;
}

// Check if new data is available
bool VL53L0X_Enhanced::IsDataReady()
{
    if (!is_initialized || !is_continuous_mode) {
        return false;
    }
    
    uint8_t ready = 0;
    VL53L0X_Error status = VL53L0X_GetMeasurementDataReady(vl53_dev, &ready);
    
    return (status == VL53L0X_ERROR_NONE && ready != 0);
}

// Perform offset calibration
bool VL53L0X_Enhanced::CalibrateOffset(uint16_t target_distance_mm)
{
    if (!is_initialized) {
        return false;
    }
    
    printf("%s: Starting offset calibration (target: %d mm)\\n", sensor_id.c_str(), target_distance_mm);
    
    int32_t offset_micro_meters = 0;
    VL53L0X_Error status = VL53L0X_PerformOffsetCalibration(vl53_dev, 
                                                           target_distance_mm * 1000,
                                                           &offset_micro_meters);
    
    if (status == VL53L0X_ERROR_NONE) {
        printf("%s: Offset calibration successful (offset: %ld um)\\n", 
               sensor_id.c_str(), offset_micro_meters);
        return true;
    }
    
    error_count++;
    printf("%s: Offset calibration failed\\n", sensor_id.c_str());
    return false;
}

// Perform crosstalk calibration
bool VL53L0X_Enhanced::CalibrateCrosstalk()
{
    if (!is_initialized) {
        return false;
    }
    
    printf("%s: Starting crosstalk calibration\\n", sensor_id.c_str());
    
    FixPoint1616_t crosstalk_compensation = 0;
    VL53L0X_Error status = VL53L0X_PerformXTalkCalibration(vl53_dev, 0, &crosstalk_compensation);
    
    if (status == VL53L0X_ERROR_NONE) {
        printf("%s: Crosstalk calibration successful\\n", sensor_id.c_str());
        return true;
    }
    
    error_count++;
    printf("%s: Crosstalk calibration failed\\n", sensor_id.c_str());
    return false;
}

// Perform complete calibration sequence
bool VL53L0X_Enhanced::PerformFullCalibration(uint16_t target_distance_mm)
{
    printf("%s: Starting full calibration sequence\\n", sensor_id.c_str());
    
    bool offset_ok = CalibrateOffset(target_distance_mm);
    bool crosstalk_ok = CalibrateCrosstalk();
    
    is_calibrated = offset_ok && crosstalk_ok;
    
    if (is_calibrated) {
        printf("%s: Full calibration completed successfully\\n", sensor_id.c_str());
    } else {
        printf("%s: Full calibration failed\\n", sensor_id.c_str());
    }
    
    return is_calibrated;
}

// Set measurement accuracy mode
bool VL53L0X_Enhanced::SetAccuracy(VL53_Accuracy accuracy)
{
    if (!is_initialized) {
        return false;
    }
    
    accuracy_mode = accuracy;
    return (ApplyConfiguration() == VL53L0X_ERROR_NONE);
}

// Set measurement mode with predefined configurations
bool VL53L0X_Enhanced::SetMeasurementMode(VL53_Accuracy mode)
{
    if (!is_initialized) {
        return false;
    }
    
    // Set timing budget based on mode
    uint32_t timing_budget;
    switch (mode) {
        case VL53_Accuracy::ULTRA_FAST:
            timing_budget = 13000;  // 13ms
            break;
        case VL53_Accuracy::FAST:
            timing_budget = 20000;  // 20ms
            break;
        case VL53_Accuracy::BETTER:
            timing_budget = 33000;  // 33ms
            break;
        case VL53_Accuracy::HIGH:
            timing_budget = 200000; // 200ms
            break;
        case VL53_Accuracy::LONG_RANGE:
            timing_budget = 300000; // 300ms
            break;
        default:
            timing_budget = 33000;  // Default to BETTER
            break;
    }
    
    // Apply timing budget first
    if (!SetTimingBudget(timing_budget)) {
        return false;
    }
    
    // Apply accuracy mode
    return SetAccuracy(mode);
}

// Set measurement timing budget
bool VL53L0X_Enhanced::SetTimingBudget(uint32_t timing_budget_us)
{
    if (!is_initialized) {
        return false;
    }
    
    VL53L0X_Error status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(vl53_dev, timing_budget_us);
    
    if (status == VL53L0X_ERROR_NONE) {
        this->timing_budget_us = timing_budget_us;
        return true;
    }
    
    error_count++;
    return false;
}

// Set signal rate threshold
bool VL53L0X_Enhanced::SetSignalThreshold(uint32_t threshold)
{
    if (!is_initialized) {
        return false;
    }
    
    FixPoint1616_t threshold_fp = (FixPoint1616_t)threshold;
    VL53L0X_Error status = VL53L0X_SetLimitCheckValue(vl53_dev,
                                                     VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                                     threshold_fp);
    
    return (status == VL53L0X_ERROR_NONE);
}

// Change sensor I2C address
bool VL53L0X_Enhanced::SetI2CAddress(uint8_t new_address)
{
    if (!is_initialized) {
        return false;
    }
    
    VL53L0X_Error status = VL53L0X_SetDeviceAddress(vl53_dev, new_address << 1);
    
    if (status == VL53L0X_ERROR_NONE) {
        i2c_address = new_address;
        vl53_dev->I2cDevAddr = new_address << 1;
        
        // Update sensor ID
        char temp_id[32];
        snprintf(temp_id, sizeof(temp_id), "VL53L0X_0x%02X", new_address);
        sensor_id = std::string(temp_id);
        
        return true;
    }
    
    error_count++;
    return false;
}

// Get sensor information string
std::string VL53L0X_Enhanced::GetSensorInfo() const
{
    char info[256];
    snprintf(info, sizeof(info),
            "%s Info:\\n"
            "I2C Address: 0x%02X\\n"
            "Initialized: %s\\n"
            "Calibrated: %s\\n"
            "Continuous Mode: %s\\n"
            "Timing Budget: %lu us\\n"
            "Error Count: %lu\\n"
            "Last Distance: %d mm\\n"
            "Last Status: %s",
            sensor_id.c_str(),
            i2c_address,
            is_initialized ? "Yes" : "No",
            is_calibrated ? "Yes" : "No",
            is_continuous_mode ? "Yes" : "No",
            timing_budget_us,
            error_count,
            last_measurement.distance_mm,
            StatusToString(last_measurement.status).c_str());
    
    return std::string(info);
}

// Reset sensor to default state
bool VL53L0X_Enhanced::Reset()
{
    printf("%s: Resetting sensor\\n", sensor_id.c_str());
    
    if (is_continuous_mode) {
        StopContinuous();
    }
    
    ResetSensor();
    HAL_Delay(10);
    
    is_initialized = false;
    is_calibrated = false;
    is_continuous_mode = false;
    
    return Init(accuracy_mode, timing_budget_us);
}

// Convert status to string
std::string VL53L0X_Enhanced::StatusToString(VL53_Status status)
{
    switch (status) {
        case VL53_Status::VALID:           return "Valid";
        case VL53_Status::OUT_OF_RANGE:    return "Out of Range";
        case VL53_Status::SIGMA_TOO_HIGH:  return "Sigma Too High";
        case VL53_Status::SIGNAL_TOO_LOW:  return "Signal Too Low";
        case VL53_Status::PHASE_ERROR:     return "Phase Error";
        case VL53_Status::HARDWARE_ERROR:  return "Hardware Error";
        case VL53_Status::NOT_INITIALIZED: return "Not Initialized";
        case VL53_Status::TIMEOUT:         return "Timeout";
        default:                           return "Unknown";
    }
}

// Private Methods Implementation

// Convert device status to simplified status
VL53_Status VL53L0X_Enhanced::ConvertDeviceStatus(uint8_t device_status)
{
    switch (device_status) {
        case 0:  return VL53_Status::VALID;
        case 1:  return VL53_Status::SIGMA_TOO_HIGH;
        case 2:  return VL53_Status::SIGNAL_TOO_LOW;
        case 3:  
        case 4:  return VL53_Status::PHASE_ERROR;
        case 5:  return VL53_Status::HARDWARE_ERROR;
        default: return VL53_Status::OUT_OF_RANGE;
    }
}

// Initialize VL53L0X device
VL53L0X_Error VL53L0X_Enhanced::InitializeDevice()
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    
    // Data initialization
    status = VL53L0X_DataInit(vl53_dev);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    // Static initialization
    status = VL53L0X_StaticInit(vl53_dev);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    // Perform reference calibration
    status = VL53L0X_PerformRefCalibration(vl53_dev, &VhvSettings, &PhaseCal);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    // Perform SPAD management
    status = VL53L0X_PerformRefSpadManagement(vl53_dev, &refSpadCount, &isApertureSpads);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    return VL53L0X_ERROR_NONE;
}

// Apply sensor configuration
VL53L0X_Error VL53L0X_Enhanced::ApplyConfiguration()
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    
    // Set timing budget
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(vl53_dev, timing_budget_us);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    // Set accuracy mode (VCSEL pulse period)
    uint8_t pre_range_period, final_range_period;
    
    switch (accuracy_mode) {
        case VL53_Accuracy::LONG_RANGE:
            pre_range_period = 18;
            final_range_period = 14;
            break;
        case VL53_Accuracy::HIGH:
            pre_range_period = 16;
            final_range_period = 12;
            break;
        case VL53_Accuracy::BETTER:
            pre_range_period = 14;
            final_range_period = 10;
            break;
        case VL53_Accuracy::FAST:
            pre_range_period = 12;
            final_range_period = 8;
            break;
        case VL53_Accuracy::ULTRA_FAST:
        default:
            pre_range_period = 10;
            final_range_period = 6;
            break;
    }
    
    status = VL53L0X_SetVcselPulsePeriod(vl53_dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, pre_range_period);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    status = VL53L0X_SetVcselPulsePeriod(vl53_dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, final_range_period);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    // Set limit checks
    status = VL53L0X_SetLimitCheckEnable(vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    status = VL53L0X_SetLimitCheckValue(vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 
                                       (FixPoint1616_t)(18 * 65536));
    if (status != VL53L0X_ERROR_NONE) return status;
    
    status = VL53L0X_SetLimitCheckEnable(vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE) return status;
    
    status = VL53L0X_SetLimitCheckValue(vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 
                                       (FixPoint1616_t)(0.25 * 65536));
    
    return status;
}

// Reset sensor using XSHUT pin
void VL53L0X_Enhanced::ResetSensor()
{
    HAL_GPIO_WritePin(xshut_port, xshut_pin, GPIO_PIN_RESET);
    HAL_Delay(100);
}

// Multi-Sensor Manager Implementation

VL53L0X_MultiSensor::VL53L0X_MultiSensor(VL53L0X_Enhanced** sensor_array, uint8_t count)
    : sensors(sensor_array), sensor_count(count), is_synchronized(false)
{
}

bool VL53L0X_MultiSensor::InitAll()
{
    bool all_success = true;
    
    printf("Initializing %d VL53L0X sensors...\\n", sensor_count);
    
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i] != nullptr) {
            bool success = sensors[i]->Init();
            if (!success) {
                all_success = false;
                printf("Sensor %d initialization failed\\n", i);
            }
        }
    }
    
    is_synchronized = all_success;
    printf("Multi-sensor initialization %s\\n", all_success ? "successful" : "failed");
    
    return all_success;
}

void VL53L0X_MultiSensor::UpdateAll()
{
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i] != nullptr && sensors[i]->IsInitialized()) {
            sensors[i]->Update();
        }
    }
}

uint8_t VL53L0X_MultiSensor::GetAllDistances(uint16_t* distances, VL53_Status* statuses)
{
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i] != nullptr && sensors[i]->IsInitialized()) {
            uint16_t distance;
            VL53_Status status;
            
            if (sensors[i]->GetDistance(distance, status)) {
                distances[i] = distance;
                if (statuses != nullptr) {
                    statuses[i] = status;
                }
                
                if (status == VL53_Status::VALID) {
                    valid_count++;
                }
            } else {
                distances[i] = 0;
                if (statuses != nullptr) {
                    statuses[i] = VL53_Status::HARDWARE_ERROR;
                }
            }
        } else {
            distances[i] = 0;
            if (statuses != nullptr) {
                statuses[i] = VL53_Status::NOT_INITIALIZED;
            }
        }
    }
    
    return valid_count;
}

bool VL53L0X_MultiSensor::StartAllContinuous(uint32_t period_ms)
{
    bool all_success = true;
    
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i] != nullptr && sensors[i]->IsInitialized()) {
            if (!sensors[i]->StartContinuous(period_ms)) {
                all_success = false;
            }
        }
    }
    
    return all_success;
}

bool VL53L0X_MultiSensor::StopAllContinuous()
{
    bool all_success = true;
    
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i] != nullptr && sensors[i]->IsInitialized()) {
            if (!sensors[i]->StopContinuous()) {
                all_success = false;
            }
        }
    }
    
    return all_success;
}

bool VL53L0X_MultiSensor::CalibrateAll(uint16_t target_distance_mm)
{
    bool all_success = true;
    
    printf("Starting calibration for all %d sensors...\\n", sensor_count);
    
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i] != nullptr && sensors[i]->IsInitialized()) {
            printf("Calibrating sensor %d...\\n", i);
            if (!sensors[i]->PerformFullCalibration(target_distance_mm)) {
                all_success = false;
            }
        }
    }
    
    printf("Multi-sensor calibration %s\\n", all_success ? "successful" : "failed");
    return all_success;
}

std::string VL53L0X_MultiSensor::GetStatusSummary() const
{
    std::string summary = "Multi-Sensor Status Summary:\\n";
    
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i] != nullptr) {
            char sensor_status[128];
            snprintf(sensor_status, sizeof(sensor_status),
                    "Sensor %d (%s): %s, Distance: %d mm\\n",
                    i,
                    sensors[i]->GetSensorID().c_str(),
                    sensors[i]->IsInitialized() ? "OK" : "FAIL",
                    sensors[i]->GetLastDistance());
            summary += std::string(sensor_status);
        } else {
            char sensor_status[64];
            snprintf(sensor_status, sizeof(sensor_status), "Sensor %d: NULL\\n", i);
            summary += std::string(sensor_status);
        }
    }
    
    return summary;
}
