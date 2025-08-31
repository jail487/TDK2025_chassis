/*
 * VL53.h
 *
 *  Created on: 2024年4月11日
 *      Author: jkasl
 */

#ifndef VL53_H_
#define VL53_H_

#include "stm32h7xx_hal.h"
#include "vl53l0x_api.h"

class VL53L0X {
private:
    VL53L0X_Dev_t vl53l0x_c;
    VL53L0X_Dev_t *vl53_dev;
    uint8_t I2C_Address; // Unique I2C address for the sensor
    GPIO_TypeDef *XSHUT_Port; // GPIO port for XSHUT pin
    uint16_t XSHUT_Pin; // GPIO pin for XSHUT

    int RangeResult; // Private member to store the range result
    VL53L0X_RangingMeasurementData_t RangingData; // Private member to store ranging data

public:
    bool isVL53_InitSuccess;

    // Constructor with proper initialization order
    VL53L0X(GPIO_TypeDef *port, uint16_t pin, uint8_t address)
        : XSHUT_Port(port), XSHUT_Pin(pin), I2C_Address(address), isVL53_InitSuccess(false), RangeResult(0) {}

    void Init();
    void Update();

    // Getter method for RangeResult
    int getRangeResult() const {
        return RangeResult;
    }
};

#endif /* VL53_H_ */
