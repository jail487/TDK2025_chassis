/*
 * VL53.c
 *
 *  Created on: 2024年4月11日
 *      Author: jkasl
 */
#include "VL53.h"
#include "vl53l0x_api.h"

extern I2C_HandleTypeDef hi2c1;

void VL53L0X::Init() {
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    // Reset sensor using XSHUT pin
    HAL_GPIO_WritePin(XSHUT_Port, XSHUT_Pin, GPIO_PIN_RESET); // XSHUT low to reset sensor
    HAL_Delay(100);

    isVL53_InitSuccess = 1;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    // Enable sensor
    HAL_GPIO_WritePin(XSHUT_Port, XSHUT_Pin, GPIO_PIN_SET); // XSHUT high to enable sensor
    HAL_Delay(100);

    vl53_dev = &vl53l0x_c;
    vl53_dev->I2cHandle = &hi2c1;
    vl53_dev->I2cDevAddr = 0x29 << 1; // Default address

    // Assign unique I2C address
    Status = VL53L0X_SetDeviceAddress(vl53_dev, I2C_Address << 1); // Use the I2C_Address member
    vl53_dev->I2cDevAddr = I2C_Address << 1;
    if (Status != VL53L0X_ERROR_NONE) {
        isVL53_InitSuccess = 0;
        printf("Sensor at address 0x%X: Failed to set I2C address\n", I2C_Address);
        return;
    }

    // Initialize sensor
    Status = VL53L0X_DataInit(vl53_dev);
    if (Status != VL53L0X_ERROR_NONE) {
        isVL53_InitSuccess = 0;
        printf("Sensor at address 0x%X: Data initialization failed\n", I2C_Address);
        return;
    }

    VL53L0X_StaticInit(vl53_dev);
    VL53L0X_PerformRefCalibration(vl53_dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(vl53_dev, &refSpadCount, &isApertureSpads);

    VL53L0X_SetLimitCheckEnable(vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536));

    VL53L0X_SetLimitCheckEnable(vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));

    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(vl53_dev, 30000);
}

void VL53L0X::Update() {
    VL53L0X_Error Status; // Declare Status as a local variable
    Status = VL53L0X_PerformSingleRangingMeasurement(vl53_dev, &RangingData);
    if (Status == VL53L0X_ERROR_NONE) {
        RangeResult = RangingData.RangeMilliMeter; // Update RangeResult
    } else {
        printf("Sensor at address 0x%X: Ranging measurement failed\n", I2C_Address);
    }
}




