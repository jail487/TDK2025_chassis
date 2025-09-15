/*
 * chassis.cpp
 *
 *  Created on: Jan 18, 2025
 *      Author: 88698
 */
#include <stdint.h>
#include "chassis.h"
#include "DC_motor.h"
#include "stm32h7xx_hal.h" // Ensure this header is included for GPIO definitions
#include <math.h>
#include "../Drivers/Pinpoint/gobilda_pinpoint_driver.h"
#include <stdio.h>
#include <cstring>  // 為 memcpy 添加此包含

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern I2C_HandleTypeDef hi2c2;
pinpoint_handle_t pinpoint;

// Pinpoint configuration values (根據實際安裝位置調整)
// 測量說明：從機器人旋轉中心到 Pinpoint 感測器的距離
// X軸：左邊為負值，右邊為正值
// Y軸：後面為負值，前面為正值
#define PINPOINT_X_OFFSET_MM    -155.0f        // 請測量實際的 X 偏移距離 (mm)
#define PINPOINT_Y_OFFSET_MM    -125.0f        // 請測量實際的 Y 偏移距離 (mm)

// Pinpoint 單位轉換因子 (如果數值是實際值的100倍，使用 0.01)
#define PINPOINT_POSITION_SCALE  0.01f      // 位置縮放因子 (調整這個值來修正單位)
#define PINPOINT_VELOCITY_SCALE  0.01f      // 速度縮放因子

// DMA 配置參數 (事件驅動模式)
#define PINPOINT_DMA_ENABLED_DEFAULT    true    // 啟用 DMA 模式
#define PINPOINT_DMA_INTERVAL_DEFAULT   10     // 最小間隔限制 (ms)，用於防止 I2C 過載

// === Chassis Physical Parameters ===
float chassis_width = 40.69;
float chassis_length = 29.55f;//轉不夠調大，太多調小
float wheel_diameter = 15.2;//cm
float span = 0.001;

// === Control Parameters ===
float kp = 4.8, ki = 10;  // 降低PID增益，減少響應激烈程度
float cmd_w_max = 1.5;
float KP_T = 0.8, KI_T = 12;  // 降低角度控制增益

// === Position and Orientation (Encoder-based) ===
float map_x = 0.f, map_y = 0.f, theta = 0.f;  // global coordinate (encoder-based)

// === Velocity Variables ===
// Real speeds from encoders (used by SpeedController)
float v_x = 0.f, v_y = 0.f, v_w = 0.f; // rps
// Command speeds (local coordinate)
float cmd_v_x = 0.f, cmd_v_y = 0.f, cmd_v_w = 0.f; // local speed  
// World coordinate speeds
float world_v_x = 0.f, world_v_y = 0.f; // world speed

// === Pinpoint Status Variables ===
bool pinpoint_initialized = false;
bool pinpoint_ready = false;
volatile bool pinpoint_dma_enabled = true;
uint32_t pinpoint_last_dma_start = 0;      // 上次 DMA 開始時間 (用於間隔控制)
uint32_t pinpoint_dma_interval = 10;       // 最小間隔限制 (ms)，防止 I2C 過載

// === External HAL Handles ===
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim24;
DC_motor wheel_FL ={&htim1,GPIOG,GPIO_PIN_11,&htim5,TIM_CHANNEL_1,4,20,1};
DC_motor wheel_FR ={&htim2,GPIOG,GPIO_PIN_14,&htim5,TIM_CHANNEL_2,4,20,1};
DC_motor wheel_BL ={&htim8,GPIOG,GPIO_PIN_15,&htim5,TIM_CHANNEL_3,4,20,0};
DC_motor wheel_BR ={&htim4,GPIOD,GPIO_PIN_15,&htim5,TIM_CHANNEL_4,4,20,1};

namespace Chassis {

// Pinpoint 狀態監控變數 (用於 Live Expressions 觀察)
pinpoint_status_t pinpoint_status = {};

void setup(){
    // 初始化所有速度變數為零，防止殘留值
    v_x = 0.0f;
    v_y = 0.0f;
    v_w = 0.0f;
    cmd_v_x = 0.0f;
    cmd_v_y = 0.0f;
    cmd_v_w = 0.0f;
    world_v_x = 0.0f;
    world_v_y = 0.0f;
    
    // 初始化 pinpoint_status 變數 (確保 Live Expressions 可以看到)
    // Chassis::pinpoint_status.init_step = 0;
    // Chassis::pinpoint_status.init_success = 0;
    // Chassis::pinpoint_status.communication_ok = 0;
    // // 注意：不要重置 device_id，讓 pinpoint_init 設置診斷代碼
    // Chassis::pinpoint_status.consecutive_failures = 0;
    
    wheel_FR.setup();
    wheel_FL.setup();
    wheel_BR.setup();
    wheel_BL.setup();
    
    // Initialize Pinpoint sensor
//    if (pinpoint_init()) {
//        // 初始化成功後，重置位置為原點
//        HAL_Delay(100);
//        pinpoint_reset_position();
//
//        // 自動配置 DMA 模式 - 暫時禁用 DMA，使用定期同步讀取
//        // pinpoint_enable_dma(PINPOINT_DMA_ENABLED_DEFAULT);     // 暫時禁用 DMA 模式
//        // pinpoint_set_dma_interval(PINPOINT_DMA_INTERVAL_DEFAULT); // 暫時禁用間隔設定
//
//        // 使用同步模式以確保穩定性
//        pinpoint_dma_enabled = false;  // 強制使用同步模式
//
//        // 立即啟動第一次 DMA 讀取，之後會由中斷回調自動鏈式觸發
//        if (pinpoint_dma_enabled) {
//            HAL_StatusTypeDef status = Pinpoint_StartDMARead(&pinpoint);
//            if (status == HAL_OK) {
//                Chassis::pinpoint_status.dma_transfers++;
//                pinpoint_last_dma_start = HAL_GetTick();
//            } else {
//                Chassis::pinpoint_status.dma_errors++;
//                // DMA 啟動失敗，記錄錯誤但保持 DMA 模式，讓回調函數重試
//                // 不禁用 DMA，讓系統自動恢復
//                static uint32_t dma_start_failures = 0;
//                dma_start_failures++;
//
//                // 只有在連續失敗很多次時才暫時切換（但很快會重新啟用）
//                if (dma_start_failures > 100) {
//                    dma_start_failures = 0;  // 重置計數器，讓系統繼續嘗試
//                }
//            }
//        }
//    }
}


void mecan_IK_transform(float _v_x, float _v_y, float _v_w) {
    // Chassis geometry factor
    float L = chassis_length / 1.0f;
    float W = chassis_width / 1.0f;
    float a = L + W;

    // Calculate wheel speeds in cm/s
    float v1 = -_v_x + _v_y + _v_w * a / 2.0f; // Front Right
    float v2 =  _v_x + _v_y - _v_w * a / 2.0f; // Front Left
    float v3 =  _v_x + _v_y + _v_w * a / 2.0f; // Back Right
    float v4 = -_v_x + _v_y - _v_w * a / 2.0f; // Back Left

    // Convert linear speed (cm/s) to RPS
    float w1 = v1 / (M_PI * wheel_diameter); // cm/s → RPS
    float w2 = v2 / (M_PI * wheel_diameter); // cm/s → RPS
    float w3 = v3 / (M_PI * wheel_diameter); // cm/s → RPS
    float w4 = v4 / (M_PI * wheel_diameter); // cm/s → RPS

    wheel_FR.setspeed(w1);
    wheel_FL.setspeed(w2);
    wheel_BR.setspeed(w3);
    wheel_BL.setspeed(w4);
}

void mecan_FK_transform() {
    // 獲取各輪子的轉速 (RPS - 每秒轉數)
    float w1 = wheel_FR.get_speed(); // RPS
    float w2 = wheel_FL.get_speed(); // RPS
    float w3 = wheel_BR.get_speed(); // RPS
    float w4 = wheel_BL.get_speed(); // RPS
    
    // 從 RPS 轉換為線速度 (cm/s)
    // 線速度 = RPS × π × 輪子直徑
    float v1 = w1 * M_PI * wheel_diameter; // Front Right (cm/s)
    float v2 = w2 * M_PI * wheel_diameter; // Front Left (cm/s)
    float v3 = w3 * M_PI * wheel_diameter; // Back Right (cm/s)
    float v4 = w4 * M_PI * wheel_diameter; // Back Left (cm/s)
    
    // 麥克納姆輪逆運動學：從輪速計算底盤速度
    float L = chassis_length; // cm
    float W = chassis_width;  // cm
    float a = L + W; // cm (與正向運動學一致)
    
    // 修正的逆運動學公式（與正向運動學一致）
    // 根據正向運動學：
    // v1 = -v_x + v_y + v_w * a/2 (Front Right)
    // v2 =  v_x + v_y - v_w * a/2 (Front Left)  
    // v3 =  v_x + v_y + v_w * a/2 (Back Right)
    // v4 = -v_x + v_y - v_w * a/2 (Back Left)
    
    v_x = (v2 + v3 - v1 - v4) / 4.0f;  // X方向線速度 (cm/s)
    v_y = (v1 + v2 + v3 + v4) / 4.0f;   // Y方向線速度 (cm/s)
    v_w = (v1 - v2 + v3 - v4) / (2.0f * a); // 角速度 (rad/s)
}


void localization() {
    // 永遠執行 encoder 定位計算 (用於 map_x, map_y, theta)
    // 方向角轉換用
    float cos_t = cos(theta);
    float sin_t = sin(theta);

    // 機體座標轉換成世界座標再積分 (encoder-based positioning)
    map_x += (v_x * cos_t - v_y * sin_t) * span;
    map_y += (v_x * sin_t + v_y * cos_t) * span;
    theta += v_w * span;
    
    // Pinpoint 定位資料會在 pinpoint_update() 中單獨更新到 pinpoint_status 中
    // 用於精度比較，不影響 map_x, map_y, theta
    
    // 可選：角度標準化在 -π ~ π，避免θ無限增長
    if (theta > M_PI) {
        theta -= 2 * M_PI;
    } else if (theta < -M_PI) {
        theta += 2 * M_PI;
    }
}

void transfer_to_localspeed(){
	cmd_v_x = (world_v_x * cos(theta)) + (world_v_y * sin(theta));
	cmd_v_y = (-world_v_x * sin(theta)) + (world_v_y * cos(theta));
}

void setSpeed(float _v_x, float _v_y, float _v_w){
    cmd_v_x = _v_x;
    cmd_v_y = _v_y;
    cmd_v_w = _v_w;
    
    // 如果設定速度為零，同時清零實際速度（防止殘留）
    if (_v_x == 0.0f && _v_y == 0.0f && _v_w == 0.0f) {
        v_x = 0.0f;
        v_y = 0.0f;
        v_w = 0.0f;
    }
}

void updateSpeed(){
    mecan_IK_transform(cmd_v_x,cmd_v_y,cmd_v_w);
    wheel_FR.PI_run();
    wheel_FL.PI_run();
    wheel_BR.PI_run();
    wheel_BL.PI_run();
    wheel_FR.updateSpeed(1);
    wheel_FL.updateSpeed(-1);
    wheel_BR.updateSpeed(1);
    wheel_BL.updateSpeed(-1);
    mecan_FK_transform();
    localization();
    
    // Pinpoint 更新已移至 DMA 中斷
    // 但在 DMA 模式停用時，仍需要備用更新
//    if (pinpoint_initialized && !pinpoint_dma_enabled) {
//        pinpoint_update();  // 僅在 DMA 停用時調用
//    }
}
}

// Pinpoint implementation functions
// void IMU_update(){
//     // 在主循環的 TIM7 中已經降頻到約 49Hz，這裡直接執行
//     // Pinpoint 更新現在頻率已經大幅降低，不會影響 1000Hz 底盤控制
    
//     if (pinpoint_initialized && !pinpoint_dma_enabled) {
//         pinpoint_update();  // 執行 Pinpoint 更新
//         imu_test++;
//     }
// }
// bool pinpoint_init() {
//     HAL_StatusTypeDef status;
//     uint8_t device_id, device_version;
    
//     // 初始化狀態變數
//     Chassis::pinpoint_status.init_step = 0;
//     Chassis::pinpoint_status.init_success = 0;
//     Chassis::pinpoint_status.device_found = 0;
//     Chassis::pinpoint_status.device_id = 0;
    
//     // 步驟 1: 檢查 I2C 是否準備就緒
//     Chassis::pinpoint_status.init_step = 1;
//     if (hi2c2.State != HAL_I2C_STATE_READY) {
//         Chassis::pinpoint_status.i2c_ready = 0;
//         return false;
//     }
//     Chassis::pinpoint_status.i2c_ready = 1;
    
//     // 步驟 2: 強制重新配置 I2C（解決 DMA 衝突）
//     Chassis::pinpoint_status.init_step = 2;
    
//     // 暫時停止 I2C
//     HAL_I2C_DeInit(&hi2c2);
//     HAL_Delay(50);
    
//     // 重新配置為標準模式，不使用 DMA
//     hi2c2.Instance = I2C2;
//     hi2c2.Init.Timing = 0x20A0C4DF;  // 使用正確的時序
//     hi2c2.Init.OwnAddress1 = 0;
//     hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//     hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//     hi2c2.Init.OwnAddress2 = 0;
//     hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//     hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//     hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
//     if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
//         Chassis::pinpoint_status.device_id = 0xF4; // I2C 重新初始化失敗
//         return false;
//     }
    
//     HAL_Delay(100); // 等待 I2C 穩定
    
//     // 步驟 3: 詳細的設備掃描測試
//     Chassis::pinpoint_status.init_step = 3;
    
//     // 先測試較短的超時時間，看看是否有任何回應
//     status = HAL_I2C_IsDeviceReady(&hi2c2, 0x31 << 1, 1, 100);
    
//     if (status == HAL_OK) {
//         Chassis::pinpoint_status.device_found = 1;
//         Chassis::pinpoint_status.device_id = 0x31;
//     } else {
//         // 嘗試不同的地址和參數
//         uint8_t test_addresses[] = {0x30, 0x31, 0x32, 0x39, 0x40, 0x48};
//         bool found = false;
        
//         for (int i = 0; i < 6; i++) {
//             status = HAL_I2C_IsDeviceReady(&hi2c2, test_addresses[i] << 1, 1, 50);
//             if (status == HAL_OK) {
//                 Chassis::pinpoint_status.device_found = 1;
//                 Chassis::pinpoint_status.device_id = test_addresses[i];
//                 found = true;
//                 break;
//             }
//         }
        
//         if (!found) {
//             // 完整掃描 I2C 總線
//             for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
//                 status = HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 20);
//                 if (status == HAL_OK) {
//                     Chassis::pinpoint_status.device_found = 1;
//                     Chassis::pinpoint_status.device_id = addr;
//                     found = true;
//                     break; // 找到第一個就停止
//                 }
//             }
//         }
        
//         if (!found) {
//             Chassis::pinpoint_status.device_found = 0;
//             // 測試 I2C 總線是否完全無回應
//             status = HAL_I2C_IsDeviceReady(&hi2c2, 0x00, 1, 10); // 無效地址
//             if (status == HAL_TIMEOUT) {
//                 Chassis::pinpoint_status.device_id = 0xE1; // 總線完全無回應
//             } else {
//                 Chassis::pinpoint_status.device_id = 0xE2; // 總線有問題但不是超時
//             }
//             return false;
//         }
//     }
    
//     // 步驟 4: 使用 Pinpoint 驅動初始化（先用阻塞模式）
//     Chassis::pinpoint_status.init_step = 4;
//     status = Pinpoint_Init(&pinpoint, &hi2c2, 0x31);
//     if (status != HAL_OK) {
//         Chassis::pinpoint_status.driver_init_ok = 0;
//         Chassis::pinpoint_status.device_id = 0xF5; // 驅動初始化失敗
//         return false;
//     }
    
//     Chassis::pinpoint_status.driver_init_ok = 1;
    
//     // 步驟 5.1: 配置編碼器設置
//     // 設置編碼器分辨率 (goBILDA 4-bar pod)
//     status = Pinpoint_SetEncoderResolution_goBILDA(&pinpoint, GOBILDA_4_BAR_POD);
//     if (status != HAL_OK) {
//         // 如果失敗，嘗試手動設置分辨率 (mm per tick)
//         Pinpoint_SetEncoderResolution(&pinpoint, 1.0f / 19.89436789f);  // 4-bar pod mm/tick
//     }
    
//     // 設置編碼器方向 (可能需要調整)
//     Pinpoint_SetEncoderDirections(&pinpoint, PINPOINT_ENCODER_FORWARD, PINPOINT_ENCODER_FORWARD);
    
//     // 設置編碼器偏移 (如果需要)
//     Pinpoint_SetOffsets(&pinpoint, 0.0f, 0.0f);  // 無偏移
    
//     // 設置 Yaw 校正係數
//     Pinpoint_SetYawScalar(&pinpoint, 1.0f);
    
//     // 強制重新校準來確保編碼器被檢測
//     HAL_Delay(50);
    
//     // 重要：重置編碼器位置到零點
//     Pinpoint_ResetPosition(&pinpoint);
    
//     // 等待一小段時間讓設備處理設置
//     HAL_Delay(100);
    
//     // 步驟 5: 讀取設備信息進行驗證
//     Chassis::pinpoint_status.init_step = 5;
//     status = Pinpoint_GetDeviceID(&pinpoint, &device_id);
//     if (status == HAL_OK) {
//         Chassis::pinpoint_status.device_id = device_id;
        
//         // 讀取設備版本
//         status = Pinpoint_GetDeviceVersion(&pinpoint, &device_version);
//         if (status == HAL_OK) {
//             Chassis::pinpoint_status.device_version = device_version;
//         }
//     }
    
//     Chassis::pinpoint_status.init_success = 1;
//     pinpoint_initialized = true;
    
//     // 步驟 6: 強制重新校準以偵測編碼器輪
//     Chassis::pinpoint_status.init_step = 6;
    
//     // 嘗試重新校準 IMU 和編碼器偵測
//     HAL_Delay(200);
//     status = Pinpoint_RecalibrateIMU(&pinpoint);
//     if (status == HAL_OK) {
//         HAL_Delay(1000); // 等待校準完成
        
//         // 重新檢查編碼器偵測狀態
//         uint8_t recalibrated_status;
//         status = Pinpoint_GetDeviceStatus(&pinpoint, &recalibrated_status);
//         if (status == HAL_OK) {
//             Chassis::pinpoint_status.device_status = recalibrated_status;
//             Chassis::pinpoint_status.x_pod_detected = (recalibrated_status & PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED) ? 0 : 1;
//             Chassis::pinpoint_status.y_pod_detected = (recalibrated_status & PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED) ? 0 : 1;
            
//             // 如果仍然未偵測到，設置診斷代碼
//             if (!Chassis::pinpoint_status.x_pod_detected || !Chassis::pinpoint_status.y_pod_detected) {
//                 if (!Chassis::pinpoint_status.x_pod_detected && !Chassis::pinpoint_status.y_pod_detected) {
//                     Chassis::pinpoint_status.device_id = 0xE6; // 校準後仍未偵測到編碼器輪
//                 } else if (!Chassis::pinpoint_status.x_pod_detected) {
//                     Chassis::pinpoint_status.device_id = 0xE7; // 校準後 X 軸編碼器輪仍未偵測
//                 } else {
//                     Chassis::pinpoint_status.device_id = 0xE8; // 校準後 Y 軸編碼器輪仍未偵測
//                 }
//             }
//         }
//     }
    
//     // 步驟 7: 配置 DMA 模式（不重新初始化 I2C）
//     Chassis::pinpoint_status.init_step = 7;
    
//     // 啟用事件驅動的 DMA 模式
//     pinpoint_enable_dma(true);
//     pinpoint_set_dma_interval(10); // 間隔參數現在用於最小間隔控制
    
//     return true;
// }

// void pinpoint_update() {
//     // DMA 事件驅動模式下，此函數僅作為同步模式備用
//     // 主要的 Pinpoint 更新已移至 HAL_I2C_MasterRxCpltCallback
    
//     // 早期返回檢查，減少不必要的計算
//     if (!pinpoint_initialized) return;
    
//     Chassis::pinpoint_status.dma_enabled = pinpoint_dma_enabled ? 1 : 0;
    
//     // 如果 DMA 模式啟用，直接返回，避免重複更新
//     if (pinpoint_dma_enabled) return;
    
//     // 檢查 I2C 是否忙碌，如果忙碌則跳過此次更新
//     if (hi2c2.State != HAL_I2C_STATE_READY) {
//         // I2C 忙碌，跳過此次更新
//         return;
//     }
    
//     // 只有在 DMA 模式被停用時，才使用同步模式
//     // 同步模式計數器
//     Chassis::pinpoint_status.total_updates++;
    
//     HAL_StatusTypeDef status;
    
//     // 優化：使用快速更新，只讀取必要的數據
//     status = Pinpoint_Update(&pinpoint);
//     if (status != HAL_OK) {
//         pinpoint_ready = false;
//         Chassis::pinpoint_status.communication_ok = 0;
//         Chassis::pinpoint_status.consecutive_failures++;
//         return;
//     }
    
//     // 快速更新基本數據，減少複雜計算
//     int32_t prev_x = Chassis::pinpoint_status.x_encoder_raw;
//     int32_t prev_y = Chassis::pinpoint_status.y_encoder_raw;
    
//     Chassis::pinpoint_status.x_encoder_raw = pinpoint.x_encoder_value;
//     Chassis::pinpoint_status.y_encoder_raw = pinpoint.y_encoder_value;
    
//     // 快速檢查變化並計數（僅基本統計）
//     if (pinpoint.x_encoder_value != prev_x || pinpoint.y_encoder_value != prev_y) {
//         Chassis::pinpoint_status.encoder_changes++;
//     }
    
//     // 更新基本位置和速度數據
//     Chassis::pinpoint_status.position_x_mm = pinpoint.current_pose.x;
//     Chassis::pinpoint_status.position_y_mm = pinpoint.current_pose.y;
//     Chassis::pinpoint_status.heading_rad = pinpoint.current_pose.heading;
//     Chassis::pinpoint_status.heading_deg = pinpoint.current_pose.heading * (180.0f / M_PI);
    
//     Chassis::pinpoint_status.velocity_x_mm_s = pinpoint.current_velocity.x_velocity;
//     Chassis::pinpoint_status.velocity_y_mm_s = pinpoint.current_velocity.y_velocity;
//     Chassis::pinpoint_status.velocity_h_rad_s = pinpoint.current_velocity.h_velocity;
    
//     // 設備狀態
//     Chassis::pinpoint_status.device_status = pinpoint.device_status;
//     Chassis::pinpoint_status.is_ready = (pinpoint.device_status & PINPOINT_STATUS_READY) ? 1 : 0;
//     Chassis::pinpoint_status.has_faults = (pinpoint.device_status & 0xFC) ? 1 : 0;
    
//     // 編碼器檢測狀態
//     Chassis::pinpoint_status.x_pod_detected = (pinpoint.device_status & PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED) ? 0 : 1;
//     Chassis::pinpoint_status.y_pod_detected = (pinpoint.device_status & PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED) ? 0 : 1;
    
//     // 成功更新
//     Chassis::pinpoint_status.successful_updates++;
//     Chassis::pinpoint_status.consecutive_failures = 0;
//     pinpoint_ready = true;
//     Chassis::pinpoint_status.communication_ok = 1;
// }

// // === Pinpoint 功能函數 ===

// bool pinpoint_get_position(float *x, float *y, float *heading) {
//     if (!pinpoint_initialized || !pinpoint_ready) return false;
    
//     pinpoint_pose_t pose;
//     HAL_StatusTypeDef status = Pinpoint_GetPosition(&pinpoint, &pose);
    
//     if (status == HAL_OK) {
//         *x = pose.x / 10.0f;     // Convert mm to cm
//         *y = pose.y / 10.0f;     // Convert mm to cm
//         *heading = pose.heading; // radians
//         return true;
//     }
    
//     return false;
// }

// bool pinpoint_set_position(float x, float y, float heading) {
//     if (!pinpoint_initialized) return false;
    
//     // Convert cm to mm
//     HAL_StatusTypeDef status = Pinpoint_SetPosition(&pinpoint, x * 10.0f, y * 10.0f, heading);
    
//     if (status == HAL_OK) {
//         printf("Pinpoint position set to: X=%.2f cm, Y=%.2f cm, H=%.2f deg\n", 
//                x, y, heading * 180.0f / M_PI);
//         return true;
//     }
    
//     return false;
// }

// bool pinpoint_reset_position() {
//     if (!pinpoint_initialized) return false;
    
//     HAL_StatusTypeDef status = Pinpoint_ResetPosition(&pinpoint);
    
//     if (status == HAL_OK) {
//         printf("Both encoder and Pinpoint position reset to origin\n");
//         // 重置 encoder 位置
//         map_x = 0.0f;
//         map_y = 0.0f;
//         theta = 0.0f;
        
//         // 重置 Pinpoint 狀態
//         Chassis::pinpoint_status.position_x_mm = 0.0f;
//         Chassis::pinpoint_status.position_y_mm = 0.0f;
//         Chassis::pinpoint_status.heading_rad = 0.0f;
//         Chassis::pinpoint_status.heading_deg = 0.0f;
        
//         return true;
//     }
    
//     return false;
// }

// bool pinpoint_is_ready() {
//     if (!pinpoint_initialized) return false;
//     return Pinpoint_IsReady(&pinpoint) && !Pinpoint_HasFaults(&pinpoint) && pinpoint_ready;
// }

// bool pinpoint_recalibrate() {
//     if (!pinpoint_initialized) return false;
    
//     printf("Recalibrating Pinpoint IMU... Keep robot stationary!\n");
//     HAL_StatusTypeDef status = Pinpoint_RecalibrateIMU(&pinpoint);
    
//     if (status == HAL_OK) {
//         // Wait for calibration to complete
//         HAL_Delay(2000);
//         printf("Pinpoint IMU recalibration completed\n");
//         return true;
//     } else {
//         printf("Pinpoint IMU recalibration failed: %d\n", status);
//         return false;
//     }
// }

// bool pinpoint_test_communication() {
//     if (!pinpoint_initialized) {
//         return false;
//     }
    
//     HAL_StatusTypeDef status;
//     uint8_t device_id, device_version, device_status;
//     pinpoint_pose_t pose;
//     pinpoint_velocity_t velocity;
    
//     Chassis::pinpoint_status.last_test_time_ms = HAL_GetTick();
    
//     // 測試 1: 基本裝置資訊讀取
//     status = Pinpoint_GetDeviceID(&pinpoint, &device_id);
//     if (status == HAL_OK) {
//         Chassis::pinpoint_status.device_id = device_id;
//     }
    
//     status = Pinpoint_GetDeviceVersion(&pinpoint, &device_version);
//     if (status == HAL_OK) {
//         Chassis::pinpoint_status.device_version = device_version;
//     }
    
//     // 測試 2: 裝置狀態檢查（詳細硬體診斷）
//     status = Pinpoint_GetDeviceStatus(&pinpoint, &device_status);
//     if (status == HAL_OK) {
//         Chassis::pinpoint_status.device_status = device_status;
//         Chassis::pinpoint_status.is_ready = Pinpoint_IsReady(&pinpoint) ? 1 : 0;
//         Chassis::pinpoint_status.has_faults = Pinpoint_HasFaults(&pinpoint) ? 1 : 0;
        
//         // 詳細的編碼器偵測狀態 - 使用正確的位元檢查
//         Chassis::pinpoint_status.x_pod_detected = (device_status & PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED) ? 0 : 1;
//         Chassis::pinpoint_status.y_pod_detected = (device_status & PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED) ? 0 : 1;
//         Chassis::pinpoint_status.imu_fault = (device_status & PINPOINT_STATUS_FAULT_IMU_RUNAWAY) ? 1 : 0;
//         Chassis::pinpoint_status.bad_read = (device_status & PINPOINT_STATUS_FAULT_BAD_READ) ? 1 : 0;
        
//         // 如果編碼器未偵測到，提供診斷資訊
//         if (!Chassis::pinpoint_status.x_pod_detected || !Chassis::pinpoint_status.y_pod_detected) {
//             // 設置診斷代碼：0xE3 = 編碼器輪未偵測
//             if (!Chassis::pinpoint_status.x_pod_detected && !Chassis::pinpoint_status.y_pod_detected) {
//                 Chassis::pinpoint_status.device_id = 0xE3; // 兩個編碼器輪都未偵測
//             } else if (!Chassis::pinpoint_status.x_pod_detected) {
//                 Chassis::pinpoint_status.device_id = 0xE4; // 僅 X 軸編碼器輪未偵測
//             } else {
//                 Chassis::pinpoint_status.device_id = 0xE5; // 僅 Y 軸編碼器輪未偵測
//             }
//         }
//     }
    
//     // 測試 3: 位置資料讀取
//     status = Pinpoint_GetPosition(&pinpoint, &pose);
//     if (status == HAL_OK) {
//         Chassis::pinpoint_status.position_x_mm = pose.x;
//         Chassis::pinpoint_status.position_y_mm = pose.y;
//         Chassis::pinpoint_status.heading_rad = pose.heading;
//         Chassis::pinpoint_status.heading_deg = pose.heading * 180.0f / M_PI;
//     }
    
//     // 測試 4: 速度資料讀取
//     status = Pinpoint_GetVelocity(&pinpoint, &velocity);
//     if (status == HAL_OK) {
//         Chassis::pinpoint_status.velocity_x_mm_s = velocity.x_velocity * PINPOINT_VELOCITY_SCALE;
//         Chassis::pinpoint_status.velocity_y_mm_s = velocity.y_velocity * PINPOINT_VELOCITY_SCALE;
//         Chassis::pinpoint_status.velocity_h_rad_s = velocity.h_velocity;
//     }
    
//     // 測試 5: 連續讀取測試
//     int success_count = 0;
//     for (int i = 0; i < 5; i++) {
//         status = Pinpoint_Update(&pinpoint);
//         if (status == HAL_OK) {
//             success_count++;
//         }
//         HAL_Delay(20);
//     }
//     Chassis::pinpoint_status.last_test_success_rate = (success_count * 100) / 5;
    
//     // 總結
//     bool communication_ok = (success_count >= 4) && Pinpoint_IsReady(&pinpoint) && !Pinpoint_HasFaults(&pinpoint);
//     Chassis::pinpoint_status.communication_ok = communication_ok ? 1 : 0;
    
//     return communication_ok;
// }

// /* DMA 控制函數實作 */

// void pinpoint_enable_dma(bool enable) {
//     pinpoint_dma_enabled = enable;
//     if (!enable) {
//         // 停用 DMA 時，確保狀態重置
//         pinpoint.dma_state = PINPOINT_DMA_IDLE;
//         pinpoint.dma_read_complete = false;
//     }
// }

// void pinpoint_set_dma_interval(uint32_t interval_ms) {
//     if (interval_ms >= 5 && interval_ms <= 1000) {  // 限制在 5ms-1000ms 之間
//         pinpoint_dma_interval = interval_ms;
//     }
// }

// bool pinpoint_is_dma_enabled() {
//     return pinpoint_dma_enabled;
// }

// void pinpoint_dma_callback() {
//     // DMA 完成回調函數，由 HAL 中斷處理函數調用
//     Pinpoint_DMACallback(&pinpoint);
// }

// /* 除錯函數 */
// uint8_t i2c_scan_devices() {
//     uint8_t devices_found = 0;
//     HAL_StatusTypeDef status;
    
//     // 重置設備 ID 紀錄
//     Chassis::pinpoint_status.device_id = 0;
    
//     // 掃描所有可能的 I2C 地址
//     for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
//         status = HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 50);
//         if (status == HAL_OK) {
//             devices_found++;
//             // 記錄第一個找到的設備地址到 device_id 以便觀察
//             if (Chassis::pinpoint_status.device_id == 0) {
//                 Chassis::pinpoint_status.device_id = addr;
//             }
//         }
//     }
    
//     return devices_found;
// }

// bool i2c_test_basic_communication() {
//     // 測試 I2C 硬體是否正常工作
//     if (hi2c2.State != HAL_I2C_STATE_READY) {
//         Chassis::pinpoint_status.device_id = 0xF0; // I2C 未準備就緒
//         return false;
//     }
    
//     // 嘗試讀取一個假地址，看看是否有正確的 NACK 回應
//     HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c2, 0x00 << 1, 1, 10);
//     if (status == HAL_TIMEOUT) {
//         Chassis::pinpoint_status.device_id = 0xF1; // I2C 總線無回應 (可能線路問題)
//         return false;
//     }
    
//     // 正常情況下應該收到 HAL_ERROR (NACK)，表示總線工作正常
//     Chassis::pinpoint_status.device_id = 0xF2; // I2C 總線正常但無設備
//     return true;
// }

///} // end namespace Chassis

// /* HAL I2C DMA 完成回調函數 - 完全事件驅動的 Pinpoint 更新 */
// extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//     // 檢查是否是我們的 I2C2 (Pinpoint)
//     if (hi2c == &hi2c2) {
//         // 防止中斷處理衝突 - 添加靜態變數檢查
//         static volatile bool callback_in_progress = false;
//         if (callback_in_progress) {
//             return;  // 如果上一次回調還在處理，直接返回避免衝突
//         }
//         callback_in_progress = true;
        
//         // 先增加回調計數器 (證明中斷有被調用)
//         Chassis::pinpoint_status.dma_completed_reads++;
        
//         // 通知 Pinpoint DMA 完成
//         Pinpoint_DMACallback(&pinpoint);
        
//         // 立即處理數據並啟動下一次讀取 (僅在 DMA 啟用時)
//         if (pinpoint_initialized && pinpoint_dma_enabled) {
//             // 更新 DMA 狀態
//             Chassis::pinpoint_status.dma_enabled = pinpoint_dma_enabled ? 1 : 0;
//             Chassis::pinpoint_status.dma_active = 1;  // DMA 正在運行
            
//             // 更新總計數器 (DMA 模式)
//             Chassis::pinpoint_status.total_updates++;
            
//             // 處理接收到的數據 - 現在包含編碼器值
//             HAL_StatusTypeDef status = Pinpoint_ProcessDMAData(&pinpoint);
//             if (status == HAL_OK) {
//                 // DMA 現在包含編碼器值，不需要額外讀取
                
//                 // 調試：複製 DMA 緩衝區前8字節用於監控
//                 memcpy(Chassis::pinpoint_status.dma_buffer_debug, pinpoint.dma_buffer, 8);
                
//                 // 調試：檢查位置數據是否全為 0xFF (可能表示未初始化的寄存器)
//                 bool all_ff = true;
//                 for (int i = 8; i < 20; i++) {  // 檢查位置數據字節 (8-19)
//                     if (pinpoint.dma_buffer[i] != 0xFF) {
//                         all_ff = false;
//                         break;
//                     }
//                 }
                
//                 // 如果位置數據全為 0xFF，可能 Pinpoint 還未完全初始化
//                 if (all_ff) {
//                     // 嘗試強制重置位置
//                     static uint32_t reset_attempts = 0;
//                     if (reset_attempts < 10) {  // 限制重置次數
//                         Pinpoint_ResetPosition(&pinpoint);
//                         reset_attempts++;
//                     }
//                 }
                
//                 // 額外測試：同步讀取位置數據進行比較
//                 static uint32_t sync_test_counter = 0;
//                 if (sync_test_counter++ < 10) {  // 只測試前10次
//                     float sync_x_pos = 0.0f;
//                     if (HAL_I2C_Mem_Read(&hi2c2, 0x31 << 1, 8, I2C_MEMADD_SIZE_8BIT, 
//                                        (uint8_t*)&sync_x_pos, 4, 50) == HAL_OK) {
//                         // 檢查同步讀取的位置是否也是 NaN
//                         if (isnan(sync_x_pos)) {
//                             // 同步讀取也是 NaN，問題在於傳感器本身
//                             // 嘗試重新配置傳感器
//                             Pinpoint_SetPosition(&pinpoint, 0.0f, 0.0f, 0.0f);
//                         }
//                     }
//                 }
                
//                 // 驗證數據有效性 - 如果位置數據是 NaN，嘗試修復
//                 if (isnan(pinpoint.current_pose.x) || isnan(pinpoint.current_pose.y) || isnan(pinpoint.current_pose.heading)) {
//                     // 位置數據無效，嘗試重置
//                     static uint32_t nan_fix_attempts = 0;
//                     if (nan_fix_attempts < 5) {
//                         Pinpoint_SetPosition(&pinpoint, 0.0f, 0.0f, 0.0f);
//                         nan_fix_attempts++;
//                     }
                    
//                     // 使用基於編碼器的簡單位置估算作為備用
//                     static float backup_x = 0.0f, backup_y = 0.0f;
//                     static int32_t last_backup_x_enc = 0, last_backup_y_enc = 0;
//                     static bool backup_initialized = false;
                    
//                     if (!backup_initialized) {
//                         last_backup_x_enc = pinpoint.x_encoder_value;
//                         last_backup_y_enc = pinpoint.y_encoder_value;
//                         backup_initialized = true;
//                     }
                    
//                     // 簡單的差分計算 (假設每個編碼器 tick 約 0.075mm)
//                     int32_t delta_x = pinpoint.x_encoder_value - last_backup_x_enc;
//                     int32_t delta_y = pinpoint.y_encoder_value - last_backup_y_enc;
                    
//                     backup_x += delta_x * 0.075f;  // 約略轉換
//                     backup_y += delta_y * 0.075f;
                    
//                     // 使用備用值
//                     pinpoint.current_pose.x = backup_x;
//                     pinpoint.current_pose.y = backup_y;
//                     pinpoint.current_pose.heading = 0.0f;  // 暫時無法計算航向
                    
//                     last_backup_x_enc = pinpoint.x_encoder_value;
//                     last_backup_y_enc = pinpoint.y_encoder_value;
//                 }
                
//                 // 修復速度數據的 NaN 問題
//                 if (isnan(pinpoint.current_velocity.x_velocity)) {
//                     pinpoint.current_velocity.x_velocity = 0.0f;
//                 }
//                 if (isnan(pinpoint.current_velocity.y_velocity)) {
//                     pinpoint.current_velocity.y_velocity = 0.0f;
//                 }
//                 if (isnan(pinpoint.current_velocity.h_velocity)) {
//                     pinpoint.current_velocity.h_velocity = 0.0f;
//                 }
                
//                 // 額外測試：強制同步讀取編碼器值以進行比較
//                 int32_t sync_x_enc = 0, sync_y_enc = 0;
//                 if (HAL_I2C_Mem_Read(&hi2c2, 0x31 << 1, 6, I2C_MEMADD_SIZE_8BIT, 
//                                    (uint8_t*)&sync_x_enc, 4, 50) == HAL_OK) {
//                     // 比較 DMA 和同步讀取的結果
//                     if (sync_x_enc != pinpoint.x_encoder_value) {
//                         // 如果不同，說明 DMA 可能有問題，使用同步值
//                         pinpoint.x_encoder_value = sync_x_enc;
//                     }
//                 }
                
//                 if (HAL_I2C_Mem_Read(&hi2c2, 0x31 << 1, 7, I2C_MEMADD_SIZE_8BIT, 
//                                    (uint8_t*)&sync_y_enc, 4, 50) == HAL_OK) {
//                     if (sync_y_enc != pinpoint.y_encoder_value) {
//                         pinpoint.y_encoder_value = sync_y_enc;
//                     }
//                 }
                
//                 // 更新成功，更新狀態
//                 Chassis::pinpoint_status.successful_updates++;
//                 Chassis::pinpoint_status.consecutive_failures = 0;
//                 pinpoint_ready = true;
//                 Chassis::pinpoint_status.communication_ok = 1;
                
//                 // 更新編碼器原始數值 (重要！)
//                 int32_t prev_x = Chassis::pinpoint_status.x_encoder_raw;
//                 int32_t prev_y = Chassis::pinpoint_status.y_encoder_raw;
                
//                 Chassis::pinpoint_status.x_encoder_raw = pinpoint.x_encoder_value;
//                 Chassis::pinpoint_status.y_encoder_raw = pinpoint.y_encoder_value;
                
//                 // 統計編碼器數據更新 (調試用) - 檢查是否有變化
//                 if (pinpoint.x_encoder_value != prev_x || pinpoint.y_encoder_value != prev_y) {
//                     // 檢查 X 軸編碼器變化是否異常大（可能導致卡死）
//                     int32_t x_delta = pinpoint.x_encoder_value - prev_x;
                    
//                     // 如果 X 軸變化異常大，可能有硬體問題，暫停 DMA 一段時間
//                     if (abs(x_delta) > 5000) {
//                         // X 軸編碼器異常，記錄錯誤但繼續運行 DMA
//                         Chassis::pinpoint_status.dma_errors++;
                        
//                         // 不禁用 DMA，讓它繼續運行
//                         // 只在嚴重錯誤時才考慮暫停
//                         static uint32_t x_axis_error_count = 0;
//                         x_axis_error_count++;
                        
//                         // 確保 DMA 保持啟用
//                         if (!pinpoint_dma_enabled) {
//                             pinpoint_dma_enabled = true;  // 強制重新啟用 DMA
//                         }
                        
//                         callback_in_progress = false;
//                         return;  // 跳過這次處理
//                     }
                    
//                     Chassis::pinpoint_status.encoder_reads_ok++;
//                     Chassis::pinpoint_status.encoder_changes++;
                    
//                     // 檢測到編碼器變化時，檢查位置是否也在更新
//                     static float last_pos_x = 0.0f, last_pos_y = 0.0f;
//                     static uint32_t encoder_changes_without_position = 0;
                    
//                     if (Chassis::pinpoint_status.encoder_changes > 10) {  // 有足夠的編碼器變化
//                         // 檢查位置是否有變化
//                         if (fabs(pinpoint.current_pose.x - last_pos_x) < 0.1f && 
//                             fabs(pinpoint.current_pose.y - last_pos_y) < 0.1f) {
//                             encoder_changes_without_position++;
                            
//                             // 如果編碼器變化但位置沒變化，嘗試重新配置
//                             if (encoder_changes_without_position > 20) {
//                                 // 嘗試重新設定編碼器分辨率和重置位置
//                                 Pinpoint_SetEncoderResolution_goBILDA(&pinpoint, GOBILDA_4_BAR_POD);
//                                 HAL_Delay(10);
//                                 Pinpoint_ResetPosition(&pinpoint);
//                                 encoder_changes_without_position = 0;  // 重置計數器
//                             }
//                         } else {
//                             encoder_changes_without_position = 0;  // 位置有變化，重置計數器
//                         }
                        
//                         last_pos_x = pinpoint.current_pose.x;
//                         last_pos_y = pinpoint.current_pose.y;
//                     }
//                 }
                
//                 // 額外調試：強制計算位置基於編碼器差值
//                 static int32_t initial_x = pinpoint.x_encoder_value;
//                 static int32_t initial_y = pinpoint.y_encoder_value;
//                 static bool initialized = false;
                
//                 if (!initialized) {
//                     initial_x = pinpoint.x_encoder_value;
//                     initial_y = pinpoint.y_encoder_value;
//                     initialized = true;
                    
//                     // 嘗試重置 Pinpoint 位置到 (0, 0, 0)
//                     Pinpoint_ResetPosition(&pinpoint);
//                 }
                
//                 // 更新位置資料
//                 pinpoint_pose_t pose = pinpoint.current_pose;
//                 pinpoint_velocity_t velocity = pinpoint.current_velocity;
                
//                 // 詳細數據監控 - 檢查所有可能的數據來源
//                 // 如果 current_pose 沒有數據，使用編碼器值進行估算
//                 if (pose.x == 0 && pose.y == 0 && pose.heading == 0) {
//                     // 使用編碼器原始值進行基本位置估算
//                     Chassis::pinpoint_status.position_x_mm = (float)pinpoint.x_encoder_value * 0.1f;
//                     Chassis::pinpoint_status.position_y_mm = (float)pinpoint.y_encoder_value * 0.1f;
//                     Chassis::pinpoint_status.heading_rad = 0.0f;  // 暫時設為0
//                 } else {
//                     // 使用 current_pose 數據
//                     Chassis::pinpoint_status.position_x_mm = pose.x;
//                     Chassis::pinpoint_status.position_y_mm = pose.y;
//                     Chassis::pinpoint_status.heading_rad = pose.heading;
//                 }
                
//                 Chassis::pinpoint_status.heading_deg = Chassis::pinpoint_status.heading_rad * 180.0f / M_PI;
                
//                 // 類似的速度數據處理
//                 if (velocity.x_velocity == 0 && velocity.y_velocity == 0) {
//                     // 嘗試從編碼器原始數據計算速度
//                     static int32_t last_x_encoder = 0, last_y_encoder = 0;
//                     static uint32_t last_time = 0;
//                     uint32_t current_time = HAL_GetTick();
                    
//                     if (last_time > 0 && (current_time - last_time) > 0) {
//                         float dt = (current_time - last_time) / 1000.0f; // 轉換為秒
//                         Chassis::pinpoint_status.velocity_x_mm_s = (pinpoint.x_encoder_value - last_x_encoder) / dt;
//                         Chassis::pinpoint_status.velocity_y_mm_s = (pinpoint.y_encoder_value - last_y_encoder) / dt;
//                     }
                    
//                     last_x_encoder = pinpoint.x_encoder_value;
//                     last_y_encoder = pinpoint.y_encoder_value;
//                     last_time = current_time;
//                 } else {
//                     Chassis::pinpoint_status.velocity_x_mm_s = velocity.x_velocity;
//                     Chassis::pinpoint_status.velocity_y_mm_s = velocity.y_velocity;
//                 }
                
//                 Chassis::pinpoint_status.velocity_h_rad_s = velocity.h_velocity;
                
//                 // 設備狀態
//                 Chassis::pinpoint_status.device_status = pinpoint.device_status;
//                 Chassis::pinpoint_status.is_ready = (pinpoint.device_status & PINPOINT_STATUS_READY) ? 1 : 0;
//                 Chassis::pinpoint_status.has_faults = (pinpoint.device_status & 0xFC) ? 1 : 0;  // 任何錯誤位
                
//                 // 檢查是否在校準模式
//                 bool is_calibrating = (pinpoint.device_status & PINPOINT_STATUS_CALIBRATING) ? true : false;
//                 if (is_calibrating) {
//                     // 如果還在校準，速度數據可能無效，設為 0
//                     pinpoint.current_velocity.x_velocity = 0.0f;
//                     pinpoint.current_velocity.y_velocity = 0.0f;
//                     pinpoint.current_velocity.h_velocity = 0.0f;
//                 }
                
//                 // 即時更新編碼器偵測狀態
//                 Chassis::pinpoint_status.x_pod_detected = (pinpoint.device_status & PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED) ? 0 : 1;
//                 Chassis::pinpoint_status.y_pod_detected = (pinpoint.device_status & PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED) ? 0 : 1;
                
//                 // 如果編碼器狀態改變，更新診斷代碼
//                 static uint8_t last_pod_status = 0xFF; // 初始化為無效值
//                 uint8_t current_pod_status = (Chassis::pinpoint_status.x_pod_detected << 1) | Chassis::pinpoint_status.y_pod_detected;
                
//                 if (current_pod_status != last_pod_status) {
//                     last_pod_status = current_pod_status;
                    
//                     switch (current_pod_status) {
//                         case 0: // 兩個都未偵測
//                             Chassis::pinpoint_status.device_id = 0xE9;
//                             break;
//                         case 1: // 僅 Y 軸偵測到
//                             Chassis::pinpoint_status.device_id = 0xEA;
//                             break;
//                         case 2: // 僅 X 軸偵測到
//                             Chassis::pinpoint_status.device_id = 0xEB;
//                             break;
//                         case 3: // 兩個都偵測到
//                             Chassis::pinpoint_status.device_id = 0x02; // 恢復正常設備 ID
//                             break;
//                     }
//                 }
                
//                 // 自動啟動下一次 DMA 讀取 (事件鏈式觸發)
//                 static uint32_t last_dma_start = 0;
//                 uint32_t current_time = HAL_GetTick();
                
//                 // 降低最小間隔以提高觸發頻率
//                 if ((current_time - last_dma_start) >= 5) {  // 改為 5ms 間隔
//                     HAL_StatusTypeDef next_status = Pinpoint_StartDMARead(&pinpoint);
//                     if (next_status == HAL_OK) {
//                         Chassis::pinpoint_status.dma_transfers++;
//                         last_dma_start = current_time;
                        
//                         // 如果 DMA 被手動關閉，在這裡重新啟用
//                         if (!pinpoint_dma_enabled) {
//                             pinpoint_dma_enabled = true;
//                         }
//                     } else {
//                         Chassis::pinpoint_status.dma_errors++;
//                         // DMA 失敗時也嘗試立即重啟
//                         HAL_StatusTypeDef retry_status = Pinpoint_StartDMARead(&pinpoint);
//                         if (retry_status == HAL_OK) {
//                             Chassis::pinpoint_status.dma_transfers++;
//                             last_dma_start = current_time;
//                         }
//                     }
//                 } else {
//                     // 即使間隔不夠，也嘗試啟動 DMA（無間隔限制測試）
//                     HAL_StatusTypeDef immediate_status = Pinpoint_StartDMARead(&pinpoint);
//                     if (immediate_status == HAL_OK) {
//                         Chassis::pinpoint_status.dma_transfers++;
//                     } else {
//                         Chassis::pinpoint_status.dma_errors++;
//                     }
//                 }
//             } else {
//                 // 數據處理失敗，記錄錯誤但繼續運行
//                 Chassis::pinpoint_status.communication_ok = 0;
//                 Chassis::pinpoint_status.consecutive_failures++;
//                 Chassis::pinpoint_status.dma_errors++;
                
//                 // 即使處理失敗，也嘗試重新啟動 DMA (自動恢復)
//                 static uint32_t last_error_retry = 0;
//                 uint32_t current_time = HAL_GetTick();
//                 if ((current_time - last_error_retry) >= 50) {  // 錯誤時間隔更長
//                     HAL_StatusTypeDef retry_status = Pinpoint_StartDMARead(&pinpoint);
//                     if (retry_status == HAL_OK) {
//                         Chassis::pinpoint_status.dma_transfers++;
//                         last_error_retry = current_time;
//                     }
//                 }
//             }
//         } else {
//             // pinpoint_initialized 為 false 時也記錄錯誤
//             Chassis::pinpoint_status.dma_errors++;
//         }
        
//         // 清除中斷處理保護標誌
//         callback_in_progress = false;
//     }
// }

// // 備用的 MasterRx 回調函數 (如果上面的不工作)
// extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//     // 直接調用 MemRx 回調函數
//     HAL_I2C_MemRxCpltCallback(hi2c);
// }

//void chassis_task(){
////	if(x<300)cmd_v_x =0.2;
////	else if (y>=300)cmd_v_x = 0;
////	if(y<300)cmd_v_y =1;
////	else if (y>=300)cmd_v_y = 0;
////	if(theta<10*pi)cmd_v_w = pi/4;
////	else if(theta>=20*pi)cmd_v_w = 0;
//
//}

/* 
 * Pinpoint 使用說明:
 * 
 * 1. 在 main() 函數中呼叫 Chassis::setup() 來初始化所有元件（包括 Pinpoint）
 * 
 * 2. 在主迴圈中呼叫 Chassis::updateSpeed() 會自動更新 Pinpoint 資料
 * 
 * 3. 直接使用全域變數 map_x, map_y, theta 取得目前位置（會自動從 Pinpoint 更新）
 * 
 * 4. 手動操作 Pinpoint:
 *    - Chassis::pinpoint_get_position(&x, &y, &heading) - 取得位置
 *    - Chassis::pinpoint_set_position(x, y, heading)    - 設定位置
 *    - Chassis::pinpoint_reset_position()               - 重設位置到原點
 *    - Chassis::pinpoint_is_ready()                     - 檢查感測器是否正常
 *    - Chassis::pinpoint_recalibrate()                  - 重新校正 IMU
 * 
 * 5. 設定參數 (在檔案開頭):
 *    - PINPOINT_X_OFFSET_MM: X軸編碼器偏移 (mm)
 *    - PINPOINT_Y_OFFSET_MM: Y軸編碼器偏移 (mm)
 *    - hi2c2: 使用的 I2C 介面
 * 
 * 6. 自動回退機制:
 *    - 如果 Pinpoint 失效，系統會自動切換回編碼器定位
 *    - 無需修改上層程式碼
 * 
 * 7. 座標系統:
 *    - Pinpoint 回傳 mm，自動轉換為 cm 來配合現有系統
 *    - 角度使用弧度制（radians）
 */





