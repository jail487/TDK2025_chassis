# 尋跡參數調整指南

## 1. 尋跡加減速參數 (pathsensor.cpp)

### 核心控制參數
```cpp
#define SPEED_CHANGE_RATE 2.0f    // 速度變化率 (每次調用增減的速度)
#define MIN_SPEED_RATIO 0.3f      // 最小速度比例 (相對於 normal_Speed)
#define LINE_LOST_THRESHOLD 1000  // 失去線條的閾值
```

### 調整建議：

#### **加速度控制**
- **SPEED_CHANGE_RATE**: 控制加減速的平滑度
  - `1.0f`: 很平滑但反應慢
  - `2.0f`: 平衡的設定 (當前值)
  - `4.0f`: 反應快但可能會震動
  - **建議範圍**: 1.0f - 5.0f

#### **最小速度控制**
- **MIN_SPEED_RATIO**: 失去線條時的最小速度比例
  - `0.1f`: 很慢，適合精確尋跡
  - `0.3f`: 中等速度 (當前值)
  - `0.5f`: 較快，適合快速通過
  - **建議範圍**: 0.1f - 0.5f

## 2. 尋跡函式設定方式

### setPath_finding_line() - 循跡直到找到線
```cpp
void setPath_finding_line(int _line_type, int _path_dir, float speed) {
    moveMode_flag = 5;
    line_type = _line_type;     // 要尋找的線條類型
    path_dir = _path_dir;       // 循跡方向 (0:前,1:後,2:右,3:左)
    move_speed = speed;         // 基礎循跡速度
    achieve_flag = false;
}
```

### setPath_distance() - 循跡指定距離
```cpp
void setPath_distance(float path_distance, int _path_dir, float speed) {
    path_dis = path_distance;   // 要移動的距離
    path_dir = _path_dir;       // 循跡方向
    move_speed = speed;         // 基礎循跡速度
    moveMode_flag = 2;
    achieve_flag = false;
}
```

## 3. 減速距離控制

### 原理說明
尋跡系統使用三階段速度控制：
1. **加速階段**: 從靜止加速到目標速度
2. **等速階段**: 維持目標速度循跡
3. **減速階段**: 接近目標時減速

### 減速距離計算
```cpp
// 在 pathsensor.cpp 的 smooth_speed_update() 函數中
// 減速距離 = (當前速度² - 最小速度²) / (2 × 減速率)

float deceleration_distance = (current_speed * current_speed - min_speed * min_speed) / 
                             (2 * SPEED_CHANGE_RATE);
```

## 4. 實際調整參數建議

### 針對不同需求的參數組合：

#### **精確尋跡** (適合需要高精度的場合)
```cpp
#define SPEED_CHANGE_RATE 1.5f    // 較慢的加減速
#define MIN_SPEED_RATIO 0.2f      // 較低的最小速度
```

#### **快速尋跡** (適合直線距離較長的場合)
```cpp
#define SPEED_CHANGE_RATE 3.0f    // 較快的加減速
#define MIN_SPEED_RATIO 0.4f      // 較高的最小速度
```

#### **平衡設定** (當前設定，適合大多數情況)
```cpp
#define SPEED_CHANGE_RATE 2.0f    // 平衡的加減速
#define MIN_SPEED_RATIO 0.3f      // 中等最小速度
```

## 5. 調整步驟建議

### 第一步：調整基礎速度
1. 在 `setPath_finding_line()` 和 `setPath_distance()` 中設定適當的 `speed` 參數
2. 建議從較低速度開始測試 (例如: 10.0f)

### 第二步：調整加減速率
1. 修改 `SPEED_CHANGE_RATE` 參數
2. 觀察機器人的加減速平滑度

### 第三步：調整最小速度
1. 修改 `MIN_SPEED_RATIO` 參數
2. 測試失去線條時的反應

### 第四步：微調減速距離
1. 根據實際測試結果微調參數
2. 確保機器人能在正確位置停止

## 6. 監控和測試

### 使用 ROS 訊息監控：
- 監控當前速度: `cmd_v_x`, `cmd_v_y`, `cmd_v_w`
- 監控線條狀態: `adcRead_ADC3[]` 數值
- 監控移動距離: `travel_distance`

### 測試場景：
1. 直線循跡 + 停止精度
2. 轉彎循跡 + 平滑度
3. 失去線條 + 恢復能力
4. 不同速度下的穩定性