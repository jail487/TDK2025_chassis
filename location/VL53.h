/*
 * VL53.h
 *
 *  Created on: 2024年4月11日
 *      Author: jkasl
 */

#ifndef INC_VL53_H_
#define INC_VL53_H_

#include "vl53l0x_api.h" // Make sure to include the API header

class VL53L0X {
public:
    VL53L0X(GPIO_TypeDef *TOF_XSHUT_PORT,
            uint16_t TOF_XSHUT_PIN,
            int VL53_ADDRESS) {
        this->TOF_XSHUT_PORT = TOF_XSHUT_PORT;
        this->TOF_XSHUT_PIN = TOF_XSHUT_PIN;
        this->VL53_ADDRESS = VL53_ADDRESS;
    }
    void Init();
    void Update();

private:
    GPIO_TypeDef *TOF_XSHUT_PORT;  
    uint16_t TOF_XSHUT_PIN;
    int VL53_ADDRESS;
    char isVL53_InitSuccess;
    int8_t VL53_Status;
    int VL53_RangeResult;
    int sub_script;
    VL53L0X_RangingMeasurementData_t RangingData;
    VL53L0X_Dev_t vl53l0x_c;
    VL53L0X_DEV vl53_dev;
};

#endif /* INC_VL53_H_ */
