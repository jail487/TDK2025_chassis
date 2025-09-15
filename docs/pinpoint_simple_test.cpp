/*
 * pinpoint_simple_test.cpp
 * 
 * 簡化的 Pinpoint 測試程式，只包含必要功能
 * 
 * Created on: Jan 18, 2025
 * Author: 88698
 */

#include "chassis.h"
#include "main.h"

// 簡單的 Pinpoint 測試函數，只用於測試，不會被編譯進主程式
void simple_pinpoint_test_example() {
    
    // 基本初始化測試
    bool init_result = Chassis::pinpoint_init();
    
    // 檢查結果透過 Live Expressions 查看：
    // pinpoint_status.init_success
    // pinpoint_status.communication_ok
    // pinpoint_status.device_id
    
    if (init_result) {
        // 詳細通訊測試
        bool comm_result = Chassis::pinpoint_test_communication();
        
        // 檢查結果透過 Live Expressions 查看：
        // pinpoint_status.last_test_success_rate
        // pinpoint_status.x_pod_detected
        // pinpoint_status.y_pod_detected
        
        // 基本位置測試
        float x, y, heading;
        bool pos_result = Chassis::pinpoint_get_position(&x, &y, &heading);
        
        // 檢查結果透過 Live Expressions 查看：
        // pinpoint_status.position_x_mm
        // pinpoint_status.position_y_mm
        // pinpoint_status.heading_deg
    }
}

/*
 * 在主程式中的使用方式：
 * 
 * int main() {
 *     // 初始化系統
 *     HAL_Init();
 *     SystemClock_Config();
 *     MX_GPIO_Init();
 *     MX_I2C2_Init();
 *     
 *     // 初始化底盤（包括 Pinpoint）
 *     Chassis::setup();
 *     
 *     // 在調試器中設置斷點，檢查 Live Expressions：
 *     // pinpoint_status.init_success      (應該是 1)
 *     // pinpoint_status.communication_ok  (應該是 1)
 *     // pinpoint_status.device_id         (應該是 48)
 *     
 *     while(1) {
 *         // 正常運行
 *         Chassis::updateSpeed(0, 0, 0);
 *         
 *         // 定期檢查狀態（透過 Live Expressions）：
 *         // pinpoint_status.consecutive_failures  (應該是 0)
 *         // pinpoint_status.position_x_mm         (位置會變化)
 *         // pinpoint_status.position_y_mm         (位置會變化)
 *         
 *         HAL_Delay(50);
 *     }
 * }
 */
