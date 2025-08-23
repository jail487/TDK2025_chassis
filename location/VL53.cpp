/*
 * VL53.c
 *
 *  Created on: 2024年4月11日
 *      Author: jkasl
 */
#include "VL53.h"

// vl530x api
#include "vl53l0x_api.h"

extern I2C_HandleTypeDef hi2c1;


//set XSHUT pin
//GPIO_TypeDef *TOF_XSHUT_PORT[3] = { GPIOA, GPIOA, GPIOC };
//const uint16_t TOF_XSHUT_PIN[3] = { GPIO_PIN_0, GPIO_PIN_12, GPIO_PIN_0 };
//
//const int VL53_ADDRESS[3] = { 0, 1, 2 };//,3 ,4 ,5 ,6 ,7};
//
//char isVL53_InitSuccess[3];
//int8_t VL53_Status[3];
//int VL53_RangeResult[3];
//int sub_script = 0;
//VL53L0X_RangingMeasurementData_t RangingData[3];
//
//VL53L0X_Dev_t vl53l0x_c[3];
//VL53L0X_DEV vl53_dev[3];

#include "VL53.h"
#include "vl53l0x_api.h"



// Single sensor pins and address
// #define TOF_XSHUT_PORT GPIOA
// #define TOF_XSHUT_PIN  GPIO_PIN_0
// #define VL53_ADDRESS   0

//char isVL53_InitSuccess = 0;
//int8_t VL53_Status = 0;
//int VL53_RangeResult = 0;
//int sub_script = 0;
//VL53L0X_RangingMeasurementData_t RangingData;
//
//VL53L0X_Dev_t vl53l0x_c;
//VL53L0X_DEV vl53_dev;

void VL53L0X::Init() {
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
	
	// Reset sensor
	vl53_dev = &vl53l0x_c;
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_Delay(100);

    isVL53_InitSuccess = 1;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    // Enable sensor
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

    vl53_dev = &vl53l0x_c;
    vl53_dev->I2cHandle = &hi2c1;
    vl53_dev->I2cDevAddr = 0x29 << 1; // Default address

    Status = VL53L0X_SetDeviceAddress(vl53_dev, (0x29 + VL53_ADDRESS + 1) << 1);
    vl53_dev->I2cDevAddr = (0x29 + VL53_ADDRESS + 1) << 1;
    if (Status != VL53L0X_ERROR_NONE) {
        isVL53_InitSuccess = 0;
        return;
    }

    Status = VL53L0X_DataInit(vl53_dev);
    if (Status != VL53L0X_ERROR_NONE) {
        isVL53_InitSuccess = 0;
        return;
    } else {
        sub_script = 100;
    }

    VL53L0X_StaticInit(vl53_dev);
    VL53L0X_PerformRefCalibration(vl53_dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(vl53_dev, &refSpadCount, &isApertureSpads);

    VL53L0X_SetLimitCheckEnable(vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(vl53_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18 * 65536));
/* Set VL53 range profiles */
		/* Change the data below base on user manual. */

		// Enable Sigma
		// Standard :      (18 * 65536)
		// High accuracy : (18 * 65536)
		// Long range :    (60 * 65536)
		// High speed :    (32 * 65536)
    VL53L0X_SetLimitCheckEnable(vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(vl53_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25 * 65536));
	// Enable Return Signal Rate
		// Standard :      (0.25 * 65536)
		// High accuracy : (0.25 * 65536)
		// Long range :    (0.10 * 65536)
		// High speed :    (0.25 * 65536)
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(vl53_dev, 30000);
	// Set the total measurement time
		// Standard :       30000
		// High accuracy : 200000
		// Long range :     33000
		// High speed :     20000
}

void VL53L0X::Update() {
	VL53_Status = VL53L0X_PerformSingleRangingMeasurement(vl53_dev, &RangingData);
    VL53_RangeResult = RangingData.RangeMilliMeter;
}
// void VL53L0X::Update() {
//     VL53_Status = VL53L0X_PerformSingleRangingMeasurement(vl53_dev, &RangingData);
//     VL53_RangeResult = RangingData.RangeMilliMeter;
// }

// void VL53_Init() {
//
// 	uint32_t refSpadCount[3];
// 	uint8_t isApertureSpads[3];
// 	uint8_t VhvSettings[3];
// 	uint8_t PhaseCal[3];
//
// 	for (size_t i = 0; i < 3; i++) {
// 		vl53_dev[i] = &vl53l0x_c[i];
// 		HAL_GPIO_WritePin(TOF_XSHUT_PORT[i], TOF_XSHUT_PIN[i], GPIO_PIN_RESET); // put XSHUT to low (Disable VL53)
// 		HAL_Delay(100);
// 	}
//
// 	for (size_t i = 0; i < 3; i++) {
// 		isVL53_InitSuccess[i] = 1;
// 		VL53L0X_Error Status = VL53L0X_ERROR_NONE;
//
// 		HAL_GPIO_WritePin(TOF_XSHUT_PORT[i], TOF_XSHUT_PIN[i], GPIO_PIN_SET); // put XSHUT to high (Enable VL53)
// 		HAL_Delay(100);
//
// 		vl53_dev[i]->I2cHandle = &hi2c1;
// 		vl53_dev[i]->I2cDevAddr = 0x29 << 1; // 0x52 -> Default address
// 		Status = VL53L0X_SetDeviceAddress(vl53_dev[i],
// 				(0x29 + VL53_ADDRESS[i] + 1) << 1);
// 		vl53_dev[i]->I2cDevAddr = (0x29 + VL53_ADDRESS[i] + 1) << 1;
// 		if (Status != VL53L0X_ERROR_NONE) {
// 			isVL53_InitSuccess[i] = 0;
// 		}
//
// 		// VL53L0X init for Single Measurement
// 		Status = VL53L0X_DataInit(vl53_dev[i]);
// 		if (Status != VL53L0X_ERROR_NONE) {
// 			isVL53_InitSuccess[i] = 0;
// 		}else{
// 			sub_script = 100;
// 		}
//
// 		VL53L0X_StaticInit(vl53_dev[i]);
// 		VL53L0X_PerformRefCalibration(vl53_dev[i], &VhvSettings[i],
// 				&PhaseCal[i]);
// 		VL53L0X_PerformRefSpadManagement(vl53_dev[i], &refSpadCount[i],
// 				&isApertureSpads[i]);
//
// 		/* Set VL53 range profiles */
// 		/* Change the data below base on user manual. */
//
// 		// Enable Sigma
// 		// Standard :      (18 * 65536)
// 		// High accuracy : (18 * 65536)
// 		// Long range :    (60 * 65536)
// 		// High speed :    (32 * 65536)
// 		VL53L0X_SetLimitCheckEnable(vl53_dev[i],
// 		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
// 		VL53L0X_SetLimitCheckValue(vl53_dev[i],
// 		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (18 * 65536));
//
// 		// Enable Return Signal Rate
// 		// Standard :      (0.25 * 65536)
// 		// High accuracy : (0.25 * 65536)
// 		// Long range :    (0.10 * 65536)
// 		// High speed :    (0.25 * 65536)
// 		VL53L0X_SetLimitCheckEnable(vl53_dev[i],
// 		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
// 		VL53L0X_SetLimitCheckValue(vl53_dev[i],
// 		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
// 				(FixPoint1616_t) (0.25 * 65536));
//
// 		// Set the total measurement time
// 		// Standard :       30000
// 		// High accuracy : 200000
// 		// Long range :     33000
// 		// High speed :     20000
// 		VL53L0X_SetMeasurementTimingBudgetMicroSeconds(vl53_dev[i], 30000);
//
// 		// Enable Long Distance
// 		// VL53L0X_SetVcselPulsePeriod(vl53_dev[i], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
// 		// VL53L0X_SetVcselPulsePeriod(vl53_dev[i], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
// 	}
//
// }

//void VL53_FirstMeasurement() {  //Correction
//	const int MeasurementTime = 3;
//	int AverageHeight[3] = { 0 };
//
//	for (size_t Time = 0; Time < MeasurementTime; Time++) {
//		for (size_t i = 0; i < 3; i++) {
//			VL53_Status[i] = VL53L0X_PerformSingleRangingMeasurement(
//					vl53_dev[i], &(RangingData[i]));
//			AverageHeight[i] += RangingData[i].RangeMilliMeter;
//		}
//	}
//	for (size_t i = 0; i < 3; i++) {
//		GroundHeight[i] = AverageHeight[i] / 3.0;
//	}
//
//	for (size_t i = 0; i < 3; i++) {
//		CakeHeightGate[i][0] = GroundHeight[i] - CAKE_HEIGHT[i] / 2;
//		CakeHeightGate[i][1] = CakeHeightGate[i][0] - CAKE_HEIGHT[i];
//		CakeHeightGate[i][2] = CakeHeightGate[i][1] - CAKE_HEIGHT[i];
//	}
//}
//
// Measurement distance
//
// void VL53_Update(){
// 	for (size_t i = 0; i < 3; i++) {
// 		VL53_Status[i] = VL53L0X_PerformSingleRangingMeasurement(vl53_dev[i], &(RangingData[i]));
// 		VL53_RangeResult[i] = RangingData[i].RangeMilliMeter;
// 	}
// }
//void VL53_Update(){
//    uint8_t DataReady = 0; // 用於儲存數據就緒狀態
//
//    for (size_t i = 0; i < 3; i++) {
//        if (isVL53_InitSuccess[i]) { // 只處理成功初始化的感測器
//            // 檢查是否有新的測量數據準備好
//            VL53L0X_Error Status = VL53L0X_GetMeasurementDataReady(vl53_dev[i], &DataReady);
//
//            if (Status == VL53L0X_ERROR_NONE && DataReady) { // 如果 API 呼叫成功且數據已準備好
//                // 獲取測量數據
//                VL53_Status[i] = VL53L0X_GetRangingMeasurementData(vl53_dev[i], &(RangingData[i]));
//
//                if (VL53_Status[i] == VL53L0X_ERROR_NONE) {
//                    VL53_RangeResult[i] = RangingData[i].RangeMilliMeter;
//                } else {
//                    // 測量出錯，可以設置一個錯誤碼或特定值
//                    VL53_RangeResult[i] = 65535; // 例如，使用 65535 表示錯誤
//                }
//                // 清除感測器內部的中斷狀態，以便感測器可以生成下一個數據就緒中斷
//                // 這非常重要，否則 DataReady 會一直為真
//                VL53L0X_ClearInterruptMask(vl53_dev[i], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
//            } else if (Status != VL53L0X_ERROR_NONE) {
//                // 獲取數據就緒狀態時發生 API 錯誤
//                VL53_Status[i] = Status; // 記錄錯誤狀態
//                VL53_RangeResult[i] = 65535; // 設置錯誤值
//            }
//            // 如果 DataReady 為 0，表示沒有新數據，VL53_RangeResult[i] 將保持舊值，VL53_Status[i] 保持舊值。
//            // 這就是非堵塞的體現，程式不會等待。
//        } else {
//            // 感測器未成功初始化，設置一個特殊值
//            VL53_RangeResult[i] = 0; // 或者一些指示未初始化錯誤的值
//            VL53_Status[i] = -1;    // 自定義錯誤碼表示未初始化
//        }
//    }
//}




