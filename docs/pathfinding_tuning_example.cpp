/*
 * pathfinding_tuning_example.cpp
 * 
 * 尋跡參數調整使用範例
 * 展示如何在運行時動態調整尋跡的加減速參數
 */

#include "location.h"
#include "pathsensor.h"

// 使用範例函式
void example_pathfinding_tuning() {
    
    // ============ 場景1：精確尋跡 ============
    // 適用於需要高精度定位的場合，如精確停在線條交叉點
    
    // 設定較慢的加減速 - 更平滑但反應較慢
    setPathfindingAcceleration(1.5f);
    
    // 設定較低的最小速度 - 失去線條時速度更慢，更容易找回
    setPathfindingMinSpeed(0.2f);
    
    // 設定較低的閾值 - 對線條檢測更敏感
    setPathfindingThreshold(800);
    
    // 執行精確尋跡
    setPath_finding_line(3, 0, 8.0f);  // 循跡直到找到十字線，速度較慢
    
    
    // ============ 場景2：快速尋跡 ============ 
    // 適用於直線距離較長，需要快速通過的場合
    
    // 設定較快的加減速 - 反應快但可能有震動
    setPathfindingAcceleration(3.5f);
    
    // 設定較高的最小速度 - 失去線條時仍保持較快速度
    setPathfindingMinSpeed(0.4f);
    
    // 設定較高的閾值 - 降低誤判，適合光線變化大的環境
    setPathfindingThreshold(1200);
    
    // 執行快速尋跡
    setPath_distance(100.0f, 0, 15.0f); // 向前循跡100cm，速度較快
    
    
    // ============ 場景3：動態調整 ============
    // 根據當前環境和需求動態調整參數
    
    float current_speed_rate, current_min_ratio;
    int current_threshold;
    
    // 獲取當前參數
    getPathfindingParams(&current_speed_rate, &current_min_ratio, &current_threshold);
    
    // 根據條件調整 (例如：光線暗時降低閾值)
    if (/* 光線檢測條件 */ false) {
        setPathfindingThreshold(current_threshold - 200);
    }
    
    // 根據速度需求調整加速度
    if (/* 需要快速通過 */ true) {
        setPathfindingAcceleration(current_speed_rate * 1.5f);
    }
    
    
    // ============ 場景4：測試和調校 ============
    // 用於測試不同參數組合的效果
    
    // 測試範圍：加速度 1.0f - 4.0f
    for (float accel = 1.0f; accel <= 4.0f; accel += 0.5f) {
        setPathfindingAcceleration(accel);
        
        // 執行測試循跡
        setPath_distance(50.0f, 0, 12.0f);
        
        // 這裡可以加入性能評估代碼
        // 例如：測量到達時間、軌跡平滑度等
        
        HAL_Delay(1000); // 暫停1秒觀察效果
    }
    
    
    // ============ 恢復預設參數 ============
    setPathfindingAcceleration(2.0f);  // 恢復預設加速度
    setPathfindingMinSpeed(0.3f);      // 恢復預設最小速度
    setPathfindingThreshold(1000);     // 恢復預設閾值
}

// 適用於不同機器人配置的預設參數組合
void apply_robot_config_preset(int robot_type) {
    switch (robot_type) {
        case 1: // 輕量型機器人
            setPathfindingAcceleration(3.0f);  // 可以快速加減速
            setPathfindingMinSpeed(0.25f);     // 較低最小速度
            setPathfindingThreshold(900);      // 較敏感的檢測
            break;
            
        case 2: // 重型機器人  
            setPathfindingAcceleration(1.5f);  // 較慢加減速避免滑動
            setPathfindingMinSpeed(0.35f);     // 較高最小速度維持動力
            setPathfindingThreshold(1100);     // 較寬鬆的檢測避免誤判
            break;
            
        case 3: // 高速機器人
            setPathfindingAcceleration(4.0f);  // 快速響應
            setPathfindingMinSpeed(0.4f);      // 高最小速度
            setPathfindingThreshold(1200);     // 寬鬆檢測適應快速移動
            break;
            
        default: // 標準配置
            setPathfindingAcceleration(2.0f);
            setPathfindingMinSpeed(0.3f);
            setPathfindingThreshold(1000);
            break;
    }
}

// 環境適應性調整
void adapt_to_environment(int light_level, int surface_type) {
    // 根據光線條件調整
    if (light_level == 0) {        // 暗光
        setPathfindingThreshold(600);   // 降低閾值，更敏感
    } else if (light_level == 1) {  // 正常光線
        setPathfindingThreshold(1000);  // 標準閾值
    } else {                       // 強光
        setPathfindingThreshold(1400);  // 提高閾值，避免反光干擾
    }
    
    // 根據地面類型調整
    if (surface_type == 0) {       // 光滑地面
        setPathfindingAcceleration(1.8f); // 減慢加速度避免滑動
        setPathfindingMinSpeed(0.25f);    // 降低最小速度
    } else if (surface_type == 1) { // 粗糙地面
        setPathfindingAcceleration(2.5f); // 可以較快加速
        setPathfindingMinSpeed(0.35f);    // 需要更多動力
    }
}