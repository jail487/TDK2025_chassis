# goBILDA Pinpoint OTOS STM32 HAL Driver

這是一個基於STM32 HAL庫的goBILDA Pinpoint OTOS（光學追蹤里程計感測器）驅動程式，適用於STM32CubeIDE專案。

## 功能特色

- **完整的I2C通信實現**：基於STM32 HAL I2C庫
- **感測器融合**：整合雙編碼器和IMU數據進行精確定位
- **易於整合**：與現有的STM32專案和ROS系統兼容
- **單位轉換**：支援多種距離和角度單位
- **錯誤處理**：完整的設備狀態監控和故障檢測
- **可配置性**：支援不同類型的goBILDA編碼器輪

## 硬件需求

- **主控制器**：STM32H7系列（或其他支援HAL庫的STM32）
- **感測器**：goBILDA Pinpoint OTOS (產品號：3110-0002-0001)
- **編碼器輪**：goBILDA 4-bar或Swingarm編碼器輪
- **連接**：I2C總線（建議使用400kHz快速模式）

## 檔案結構

```
location/
├── gobilda_pinpoint_driver.h    # 驅動頭文件
├── gobilda_pinpoint_driver.c    # 驅動實現
└── pinpoint_example.c           # 使用示例
```

## 快速開始

### 1. 硬件連接

```
STM32 I2C1          Pinpoint OTOS
-----------         -------------
VCC (3.3V)    <-->  VCC
GND           <-->  GND
SCL (PB8)     <-->  SCL
SDA (PB9)     <-->  SDA
```

### 2. STM32CubeIDE配置

1. **啟用I2C1**：
   - 在CubeMX中配置I2C1
   - 設定速度為400kHz (Fast Mode)
   - 啟用DMA（可選，提升性能）

2. **添加驅動文件**：
   - 將 `gobilda_pinpoint_driver.h` 和 `gobilda_pinpoint_driver.c` 添加到專案
   - 在Include路徑中添加驅動頭文件目錄

### 3. 基本使用

```c
#include "gobilda_pinpoint_driver.h"

extern I2C_HandleTypeDef hi2c1;
pinpoint_handle_t pinpoint;

// 初始化
HAL_StatusTypeDef status = Pinpoint_Init(&pinpoint, &hi2c1, PINPOINT_DEFAULT_I2C_ADDR);

// 配置編碼器輪
Pinpoint_SetEncoderResolution_goBILDA(&pinpoint, GOBILDA_4_BAR_POD);

// 設定偏移量（根據機器人幾何結構調整）
Pinpoint_SetOffsets(&pinpoint, -84.0f, -168.0f);

// 主循環中更新感測器數據
while(1) {
    Pinpoint_Update(&pinpoint);
    
    pinpoint_pose_t pose;
    Pinpoint_GetPosition(&pinpoint, &pose);
    
    printf("Position: X=%.2f mm, Y=%.2f mm, Heading=%.2f rad\n", 
           pose.x, pose.y, pose.heading);
           
    HAL_Delay(20); // 50Hz更新率
}
```

## API 參考

### 初始化函數

- `Pinpoint_Init()` - 初始化驅動
- `Pinpoint_Reset()` - 重置設備
- `Pinpoint_SetOffsets()` - 設定編碼器輪偏移
- `Pinpoint_SetEncoderResolution_goBILDA()` - 設定goBILDA編碼器輪類型

### 數據讀取

- `Pinpoint_Update()` - 更新所有感測器數據
- `Pinpoint_GetPosition()` - 獲取當前位置
- `Pinpoint_GetVelocity()` - 獲取當前速度

### 狀態監控

- `Pinpoint_IsReady()` - 檢查設備是否就緒
- `Pinpoint_HasFaults()` - 檢查是否有故障
- `Pinpoint_GetDeviceStatus()` - 獲取詳細狀態

### 校準與重置

- `Pinpoint_RecalibrateIMU()` - 重新校準IMU
- `Pinpoint_ResetPosition()` - 重置位置到原點
- `Pinpoint_SetPosition()` - 手動設定位置

## 與現有系統整合

### 與ROS系統整合

```c
// 在ROS發布函數中
void pub_chassis_pose(void) {
    pinpoint_pose_t pose;
    if (Pinpoint_GetPosition(&pinpoint, &pose) == HAL_OK) {
        chassis_current_pose.position.x = pose.x / 1000.0f; // mm轉m
        chassis_current_pose.position.y = pose.y / 1000.0f;
        // 將heading轉換為四元數...
        pub_chassis.publish(&chassis_current_pose);
    }
}
```

### 與底盤控制系統整合

```c
// 在chassis.cpp中
extern pinpoint_handle_t pinpoint;

void Chassis::updateOdometry() {
    Pinpoint_Update(&pinpoint);
    
    pinpoint_pose_t pose;
    Pinpoint_GetPosition(&pinpoint, &pose);
    
    // 更新全域位置變數
    map_x = pose.x;
    map_y = pose.y;
    theta = pose.heading;
}
```

## 配置參數

### 編碼器輪偏移

根據用戶手冊3110-0002-0001，推薦配置：

```c
// X pod偏移：側向編碼器輪相對於追蹤點的距離
// 左側為正值，右側為負值
#define X_POD_OFFSET    -84.0f   // mm

// Y pod偏移：前向編碼器輪相對於追蹤點的距離  
// 前方為正值，後方為負值
#define Y_POD_OFFSET    -168.0f  // mm
```

### 編碼器輪類型

```c
// goBILDA 4-bar編碼器輪（推薦）
Pinpoint_SetEncoderResolution_goBILDA(&pinpoint, GOBILDA_4_BAR_POD);

// goBILDA Swingarm編碼器輪
Pinpoint_SetEncoderResolution_goBILDA(&pinpoint, GOBILDA_SWINGARM_POD);

// 自定義編碼器輪
Pinpoint_SetEncoderResolution(&pinpoint, 0.05037f); // mm/tick
```

## 故障排除

### 常見問題

1. **設備無法初始化**
   - 檢查I2C連接和電源
   - 確認I2C地址正確（默認0x31）
   - 檢查I2C時鐘配置

2. **編碼器輪無法檢測**
   - 確認編碼器輪正確連接
   - 檢查編碼器輪方向設定
   - 驗證編碼器輪類型配置

3. **IMU數據異常**
   - 執行IMU重新校準
   - 確保設備在校準時保持靜止
   - 檢查磁場干擾

### 調試技巧

```c
// 啟用詳細狀態監控
uint8_t status;
Pinpoint_GetDeviceStatus(&pinpoint, &status);

if (status & PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED) {
    printf("X編碼器輪未檢測到\n");
}
if (status & PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED) {
    printf("Y編碼器輪未檢測到\n");
}
if (status & PINPOINT_STATUS_FAULT_IMU_RUNAWAY) {
    printf("IMU數據異常\n");
}
```

## 性能建議

- **更新頻率**：建議50-100Hz更新感測器數據
- **I2C速度**：使用400kHz快速模式以獲得最佳性能
- **DMA**：啟用I2C DMA可以減少CPU負載
- **濾波**：如果需要，可以在應用層添加卡爾曼濾波器

## 授權

此驅動程式基於MIT授權，與原始的goBILDA FTC驅動程式保持兼容。

## 技術支持

如需技術支持，請聯繫：
- goBILDA官方：tech@gobilda.com
- 專案維護者：[您的聯繫信息]

---

**注意**：使用前請仔細閱讀goBILDA Pinpoint OTOS用戶手冊，確保正確的硬件配置和安全操作。
