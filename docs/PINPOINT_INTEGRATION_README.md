# Pinpoint 整合設定說明

## 檔案結構

在 `chassis.cpp` 中已完整實作 Pinpoint 定位功能：

```
TDK2025_chassis/
├── Chassis/
│   ├── chassis.h              # 包含 Pinpoint 函數宣告
│   ├── chassis.cpp            # 完整 Pinpoint 實作
│   └── pinpoint_usage_example.cpp # 使用範例
└── location/
    ├── gobilda_pinpoint_driver.h  # Pinpoint 驅動程式標頭檔
    ├── gobilda_pinpoint_driver.c  # Pinpoint 驅動程式實作
    └── pinpoint_example.cpp      # 原始範例檔案
```

## 編譯設定

確認以下設定：

### 1. Include Path
在 STM32CubeIDE 中加入以下 include path：
- `../location` （相對於 Chassis 資料夾）

### 2. Source Files
確認以下檔案被包含在編譯中：
- `Chassis/chassis.cpp`
- `location/gobilda_pinpoint_driver.c`

### 3. I2C 設定
在 STM32CubeMX 中設定：
- 啟用 I2C2 (100kHz 或 400kHz)
- 設定適當的 GPIO pins

## 使用方式

### 基本使用
```cpp
#include "chassis.h"

int main() {
    // 初始化所有底盤元件（包括 Pinpoint）
    Chassis::setup();
    
    while(1) {
        // 正常底盤控制（會自動更新 Pinpoint）
        Chassis::updateSpeed(vx, vy, vw);
        
        // 使用全域位置變數
        extern float map_x, map_y, theta;
        printf("Position: %.2f, %.2f, %.2f\n", map_x, map_y, theta);
        
        HAL_Delay(20);
    }
}
```

### 進階功能
```cpp
// 檢查 Pinpoint 狀態
if (Chassis::pinpoint_is_ready()) {
    // Pinpoint 正常工作
}

// 手動取得位置
float x, y, heading;
Chassis::pinpoint_get_position(&x, &y, &heading);

// 設定位置
Chassis::pinpoint_set_position(100.0f, 50.0f, M_PI/4);

// 重設位置
Chassis::pinpoint_reset_position();

// 重新校正 IMU
Chassis::pinpoint_recalibrate();
```

## 設定參數

在 `chassis.cpp` 開頭調整：

```cpp
// Pinpoint 編碼器偏移（根據你的機器人調整）
#define PINPOINT_X_OFFSET_MM    -84.0f   
#define PINPOINT_Y_OFFSET_MM    -168.0f  

// I2C 介面
extern I2C_HandleTypeDef hi2c2;  // 改為你使用的 I2C
```

## 自動回退機制

- 如果 Pinpoint 初始化失敗或發生故障，系統會自動切換回原本的編碼器定位
- 上層程式碼不需要修改，`map_x`, `map_y`, `theta` 變數會持續更新
- 可以用 `Chassis::pinpoint_is_ready()` 檢查 Pinpoint 狀態

## 座標系統

- **輸入/輸出單位**: cm（公分）
- **內部 Pinpoint**: mm（公釐），自動轉換
- **角度**: radians（弧度）
- **速度**: cm/s, rad/s

## 除錯資訊

Pinpoint 會透過 printf 輸出除錯資訊：
- 初始化狀態
- 裝置 ID 和版本
- 錯誤訊息
- 定期位置資訊（每5秒一次）

## 確認通訊成功

### 方法 1: 使用詳細初始化測試
```cpp
#include "chassis.h"

int main() {
    // 初始化底盤（包含詳細的 Pinpoint 測試）
    Chassis::setup();  // 會自動顯示詳細的初始化過程
    
    // 檢查結果
    if (Chassis::pinpoint_is_ready()) {
        printf("✅ Pinpoint 通訊成功！\n");
    } else {
        printf("❌ Pinpoint 通訊失敗！\n");
    }
}
```

### 方法 2: 使用專門的通訊測試
```cpp
// 完整的通訊測試
bool success = Chassis::pinpoint_test_communication();
if (success) {
    printf("🎉 所有測試通過！\n");
} else {
    printf("⚠️  請檢查硬體連接\n");
}
```

### 方法 3: 監控 printf 輸出
正常的初始化會顯示：
```
=== Pinpoint 通訊測試開始 ===
1. 檢查 I2C2 狀態...
   OK: I2C2 準備就緒
2. 掃描 I2C 裝置...
   發現裝置在地址: 0x30
   *** 這是 Pinpoint 預設地址! ***
...
=== Pinpoint 初始化完全成功! ===
```

### 通訊成功的指標

✅ **成功指標**:
- Device ID 讀取成功 (通常是 0x30)
- 裝置狀態正常 (0x00)
- 位置資料可以正常讀取
- 連續讀取成功率 > 90%

❌ **失敗指標**:
- I2C 掃描找不到裝置
- Device ID 是 0x00 或 0xFF
- 裝置狀態有錯誤位元
- 位置讀取持續失敗

## 故障排除

### 常見問題與解決方法

1. **找不到 I2C 裝置**
   ```
   ERROR: 在預設地址 0x30 找不到 Pinpoint 裝置
   ```
   **檢查**: I2C 線路、電源、上拉電阻

2. **裝置 ID 異常**
   ```
   Device ID: 0x00 或 0xFF
   ```
   **檢查**: I2C 時脈太快、線路干擾

3. **編碼器未偵測**
   ```
   ⚠️  X軸編碼器未偵測到
   ⚠️  Y軸編碼器未偵測到
   ```
   **檢查**: 編碼器 pods 連接、電源

4. **IMU 問題**
   ```
   ⚠️  IMU 失控
   ```
   **解決**: 呼叫 `Chassis::pinpoint_recalibrate()`

### 效能調整

- 主迴圈頻率：20-50 Hz（HAL_Delay(20-50)）
- Pinpoint 更新會在 `Chassis::updateSpeed()` 中自動執行
- 位置除錯輸出每5秒一次，避免過多輸出

## 範例檔案

詳細使用範例請參考：
- `pinpoint_usage_example.cpp` - 完整使用範例
- `pinpoint_example.cpp` - 原始驅動程式範例
