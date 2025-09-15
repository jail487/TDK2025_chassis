/*
 * chassis.h
 *
 *  Created on: Jan 18, 2025
 *      Author: 88698
 */

#ifndef CHASSIS_H_
#define CHASSIS_H_

#include <stdint.h>  // 包含 uint8_t, uint32_t 等定義

// === Chassis Physical Parameters ===
extern float chassis_width;
extern float chassis_length;
extern float wheel_diameter;
extern float span;

// === Control Parameters ===
extern float kp, ki;
extern float KP_T, KI_T;
extern float cmd_w_max;

// === Position and Orientation (Encoder-based) ===
extern float map_x, map_y, theta;  // encoder-based positioning (cm, cm, rad)

// === Velocity Variables ===
// Real speeds from encoders (used by SpeedController)
extern float v_x, v_y, v_w;  // rps
// Command speeds (local coordinate)
extern float cmd_v_x, cmd_v_y, cmd_v_w;  // local speed
// World coordinate speeds
extern float world_v_x, world_v_y;  // world speed

// === Pinpoint Status Variables ===
extern bool pinpoint_initialized;
extern bool pinpoint_ready;
extern volatile bool pinpoint_dma_enabled;
extern uint32_t pinpoint_last_dma_start;
extern uint32_t pinpoint_dma_interval;

// === Mathematical Constants ===
#define pi 3.14159265358979323846

namespace Chassis {

void setup();
void mecan_IK_transform(float _v_x,float _v_y,float _v_w);
void mecan_FK_transform();
void localization();
void updateSpeed();
void setSpeed(float _v_x, float _v_y, float _v_w);  // 設定速度指令
void chassis_task();

// Pinpoint functions
bool pinpoint_init();
void pinpoint_update();
bool pinpoint_get_position(float *x, float *y, float *heading);
bool pinpoint_set_position(float x, float y, float heading);
bool pinpoint_reset_position();
bool pinpoint_is_ready();
bool pinpoint_recalibrate();
bool pinpoint_test_communication();  // 測試通訊功能

// DMA 控制函數
void pinpoint_enable_dma(bool enable);     // 啟用/停用 DMA 模式
void pinpoint_set_dma_interval(uint32_t interval_ms);  // 設定 DMA 讀取間隔
bool pinpoint_is_dma_enabled();            // 檢查 DMA 是否啟用
void pinpoint_dma_callback();              // DMA 完成回調函數

// 除錯函數
uint8_t i2c_scan_devices();               // 掃描 I2C 設備，返回找到的設備數量
bool i2c_test_basic_communication();      // 測試基本 I2C 通訊功能

// Pinpoint 狀態監控變數 (用於 Live Expressions)
typedef struct {
    // 初始化狀態
    uint8_t init_step;              // 目前初始化步驟 (0-11)
    uint8_t init_success;           // 初始化是否成功 (0=失敗, 1=成功)
    uint8_t i2c_ready;              // I2C 是否準備就緒 (0=否, 1=是)
    uint8_t device_found;           // 是否找到裝置 (0=否, 1=是)
    uint8_t driver_init_ok;         // 驅動程式初始化是否成功
    
    // 裝置資訊
    uint8_t device_id;              // Pinpoint Device ID
    uint8_t device_version;         // Pinpoint Version
    uint8_t device_status;          // 裝置狀態暫存器
    
    // 運行狀態
    uint8_t is_ready;               // 裝置是否準備就緒
    uint8_t has_faults;             // 是否有錯誤
    uint8_t communication_ok;       // 通訊是否正常
    uint8_t consecutive_failures;   // 連續失敗次數
    
    // 位置資料
    float position_x_mm;            // X 位置 (mm)
    float position_y_mm;            // Y 位置 (mm)
    float heading_rad;              // 航向 (radians)
    float heading_deg;              // 航向 (degrees)
    
    // 速度資料
    float velocity_x_mm_s;          // X 速度 (mm/s)
    float velocity_y_mm_s;          // Y 速度 (mm/s)
    float velocity_h_rad_s;         // 角速度 (rad/s)
    
    // 錯誤診斷
    uint8_t x_pod_detected;         // X軸編碼器是否偵測到
    uint8_t y_pod_detected;         // Y軸編碼器是否偵測到
    uint8_t imu_fault;              // IMU 是否有錯誤
    uint8_t bad_read;               // 是否有讀取錯誤
    
    // 通訊測試結果
    uint8_t last_test_success_rate; // 最後一次測試的成功率 (%)
    uint32_t last_test_time_ms;     // 最後一次測試時間戳
    uint32_t total_updates;         // 總更新次數
    uint32_t successful_updates;    // 成功更新次數
    
    // DMA 狀態監控
    uint8_t dma_enabled;            // DMA 是否啟用
    uint8_t dma_active;             // DMA 是否正在執行
    uint32_t dma_transfers;         // DMA 傳輸次數
    uint32_t dma_errors;            // DMA 錯誤次數
    uint8_t dma_state;              // DMA 當前狀態 (0=IDLE, 1=READING, 2=COMPLETE, 3=ERROR)
    uint32_t dma_completed_reads;   // DMA 完成的讀取次數
    
    // 編碼器原始數值監控
    int32_t x_encoder_raw;          // X 軸編碼器原始數值
    int32_t y_encoder_raw;          // Y 軸編碼器原始數值
    uint32_t encoder_reads_ok;      // 編碼器讀取成功次數
    uint32_t encoder_changes;       // 編碼器值變化次數
    
    // DMA 緩衝區調試
    uint8_t dma_buffer_debug[8];    // DMA 緩衝區前8字節 (編碼器數據)
    
    // 編碼器測試和調試
    uint32_t encoder_test_counter;  // 編碼器測試計數器
    int32_t encoder_max_x;          // 記錄的最大 X 編碼器值
    int32_t encoder_max_y;          // 記錄的最大 Y 編碼器值
    int32_t encoder_min_x;          // 記錄的最小 X 編碼器值
    int32_t encoder_min_y;          // 記錄的最小 Y 編碼器值
    
} pinpoint_status_t;

extern pinpoint_status_t pinpoint_status;

}  // end namespace Chassis









#endif /* CHASSIS_H_ */
