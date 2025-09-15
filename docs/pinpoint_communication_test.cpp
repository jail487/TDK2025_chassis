/*
 * pinpoint_communication_test.cpp
 * 
 * Pinpoint 通訊測試指南
 * 
 * 這個檔案展示如何確認 Pinpoint 與 STM32 的通訊是否成功
 * 
 * Created on: Jan 18, 2025
 * Author: 88698
 */

#include "chassis.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 函數宣告
void real_time_monitoring_test();
void hardware_troubleshooting_guide();
void pinpoint_health_check();
bool quick_pinpoint_test();

/* ========== 在主程式中的測試方式 ========== */

void main_communication_test() {
    
    printf("\n🔧 Pinpoint 通訊測試開始...\n");
    printf("=====================================\n");
    
    // 方法 1: 基本初始化測試（已包含詳細檢測）
    printf("\n📋 步驟 1: 初始化測試\n");
    bool init_success = Chassis::pinpoint_init();
    
    if (init_success) {
        printf("✅ 初始化測試通過\n");
        
        // 方法 2: 詳細通訊測試
        printf("\n📋 步驟 2: 詳細通訊測試\n");
        bool comm_test = Chassis::pinpoint_test_communication();
        
        if (comm_test) {
            printf("\n🎉 所有測試通過！Pinpoint 可以正常使用\n");
            
            // 方法 3: 實時監控測試
            printf("\n📋 步驟 3: 實時監控測試 (10秒)\n");
            real_time_monitoring_test();
            
        } else {
            printf("\n⚠️  通訊測試失敗，請檢查硬體\n");
            hardware_troubleshooting_guide();
        }
        
    } else {
        printf("\n❌ 初始化失敗\n");
        hardware_troubleshooting_guide();
    }
    
    printf("\n=====================================\n");
    printf("🔧 Pinpoint 通訊測試結束\n");
}

/* ========== 實時監控測試 ========== */

void real_time_monitoring_test() {
    printf("開始 10 秒實時監控...\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t last_print = 0;
    int update_count = 0;
    int success_count = 0;
    
    while ((HAL_GetTick() - start_time) < 10000) {  // 10 seconds
        
        // 更新 Pinpoint
        Chassis::setSpeed(0, 0, 0);  // 靜止狀態
        Chassis::updateSpeed();
        
        // 每 500ms 顯示一次狀態
        if ((HAL_GetTick() - last_print) > 500) {
            
            update_count++;
            
            if (Chassis::pinpoint_is_ready()) {
                success_count++;
                
                float x, y, heading;
                if (Chassis::pinpoint_get_position(&x, &y, &heading)) {
                    printf("  [%d] ✅ X:%.1fcm Y:%.1fcm H:%.1f° (成功率:%d%%)\n", 
                           update_count, x, y, heading * 180.0f / M_PI,
                           (success_count * 100) / update_count);
                } else {
                    printf("  [%d] ⚠️  位置讀取失敗\n", update_count);
                }
            } else {
                printf("  [%d] ❌ Pinpoint 未準備就緒\n", update_count);
            }
            
            last_print = HAL_GetTick();
        }
        
        HAL_Delay(50);  // 20Hz 更新
    }
    
    int final_success_rate = (success_count * 100) / update_count;
    printf("\n監控結果:\n");
    printf("  總更新次數: %d\n", update_count);
    printf("  成功次數: %d\n", success_count);
    printf("  成功率: %d%%\n", final_success_rate);
    
    if (final_success_rate >= 90) {
        printf("  ✅ 通訊品質優秀\n");
    } else if (final_success_rate >= 70) {
        printf("  ⚠️  通訊品質一般，建議檢查連接\n");
    } else {
        printf("  ❌ 通訊品質差，需要檢修\n");
    }
}

/* ========== 硬體檢查指南 ========== */

void hardware_troubleshooting_guide() {
    printf("\n🔧 硬體檢查指南:\n");
    printf("=====================================\n");
    
    printf("1. 電源檢查:\n");
    printf("   - Pinpoint 是否有電源指示燈？\n");
    printf("   - 電壓是否在 3.3V-5V 範圍內？\n");
    printf("   - 電源線是否牢固連接？\n\n");
    
    printf("2. I2C 連接檢查:\n");
    printf("   - SDA 線是否連接到正確的 GPIO pin？\n");
    printf("   - SCL 線是否連接到正確的 GPIO pin？\n");
    printf("   - 是否有 4.7kΩ 上拉電阻？\n");
    printf("   - I2C 線路長度是否過長？(建議 < 30cm)\n\n");
    
    printf("3. STM32 I2C 設定檢查:\n");
    printf("   - STM32CubeMX 中 I2C2 是否已啟用？\n");
    printf("   - I2C 時脈設定是否正確？(建議 100kHz)\n");
    printf("   - GPIO 設定是否為 Open Drain + Pull-up？\n\n");
    
    printf("4. Pinpoint 編碼器檢查:\n");
    printf("   - X 軸編碼器是否正確連接？\n");
    printf("   - Y 軸編碼器是否正確連接？\n");
    printf("   - 編碼器 pods 是否安裝牢固？\n");
    printf("   - 編碼器線是否損壞？\n\n");
    
    printf("5. 軟體設定檢查:\n");
    printf("   - 包含路徑是否正確？(-I../location)\n");
    printf("   - gobilda_pinpoint_driver.c 是否被編譯？\n");
    printf("   - I2C 地址是否正確？(預設 0x30)\n");
    printf("   - printf 輸出是否被重定向？\n\n");
    
    printf("建議檢查順序:\n");
    printf("1. 先用萬用表確認電源和 I2C 線路\n");
    printf("2. 用示波器檢查 I2C 信號\n");
    printf("3. 檢查 STM32CubeMX 設定\n");
    printf("4. 確認編碼器硬體連接\n");
    printf("=====================================\n");
}

/* ========== 定期健康檢查 ========== */

void pinpoint_health_check() {
    static uint32_t last_health_check = 0;
    static int consecutive_failures = 0;
    
    // 每 5 秒檢查一次
    if ((HAL_GetTick() - last_health_check) > 5000) {
        
        if (Chassis::pinpoint_is_ready()) {
            if (consecutive_failures > 0) {
                printf("✅ Pinpoint 已恢復正常 (曾連續失敗 %d 次)\n", consecutive_failures);
                consecutive_failures = 0;
            }
        } else {
            consecutive_failures++;
            printf("⚠️  Pinpoint 健康檢查失敗 (連續 %d 次)\n", consecutive_failures);
            
            if (consecutive_failures >= 3) {
                printf("🚨 Pinpoint 可能需要維修！\n");
                // 可以在這裡加入自動重新初始化或安全模式
            }
        }
        
        last_health_check = HAL_GetTick();
    }
}

/* ========== 簡單的快速測試 ========== */

bool quick_pinpoint_test() {
    printf("🔍 Pinpoint 快速測試...\n");
    
    // 檢查是否已初始化
    if (!Chassis::pinpoint_is_ready()) {
        printf("❌ Pinpoint 未準備就緒\n");
        return false;
    }
    
    // 嘗試讀取位置
    float x, y, heading;
    if (!Chassis::pinpoint_get_position(&x, &y, &heading)) {
        printf("❌ 無法讀取位置\n");
        return false;
    }
    
    printf("✅ 快速測試通過 - 位置: (%.1f, %.1f, %.1f°)\n", 
           x, y, heading * 180.0f / M_PI);
    return true;
}

/* ========== 使用範例 ========== */

/*
 * 在你的 main.cpp 中使用方式:
 * 
 * int main() {
 *     // 系統初始化...
 *     
 *     // 完整測試 (第一次使用或出問題時)
 *     main_communication_test();
 *     
 *     // 或者只做快速測試
 *     // if (!quick_pinpoint_test()) {
 *     //     Error_Handler();
 *     // }
 *     
 *     while(1) {
 *         // 正常程式...
 *         Chassis::updateSpeed(vx, vy, vw);
 *         
 *         // 定期健康檢查
 *         pinpoint_health_check();
 *         
 *         HAL_Delay(50);
 *     }
 * }
 */
