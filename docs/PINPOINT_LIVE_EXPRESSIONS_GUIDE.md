# STM32CubeIDE Live Expressions 監控指南

## Pinpoint 狀態監控變數

在 STM32CubeIDE 的 Live Expressions 中加入以下變數來監控 Pinpoint 狀態：

### 🔧 初始化監控

```
pinpoint_status.init_step          // 目前初始化步驟 (0-11)
pinpoint_status.init_success        // 初始化成功 (1=成功, 0=失敗)
pinpoint_status.i2c_ready          // I2C 準備就緒 (1=是, 0=否)
pinpoint_status.device_found        // 找到裝置 (1=是, 0=否)
pinpoint_status.driver_init_ok      // 驅動初始化成功 (1=是, 0=否)
```

**預期值**：
- `init_step`: 正常初始化會從 0 → 11
- `init_success`: 成功時應該是 1
- `i2c_ready`: 應該是 1
- `device_found`: 應該是 1
- `driver_init_ok`: 應該是 1

### 📡 裝置資訊監控

```
pinpoint_status.device_id           // 裝置 ID (應該是 0x30)
pinpoint_status.device_version      // 裝置版本
pinpoint_status.device_status       // 裝置狀態 (0=正常)
```

**預期值**：
- `device_id`: 0x30 (48 十進位)
- `device_version`: 任何非 0xFF 的值
- `device_status`: 0 表示正常

### 🔄 運行狀態監控

```
pinpoint_status.is_ready            // 裝置準備就緒 (1=是, 0=否)
pinpoint_status.has_faults          // 有錯誤 (0=正常, 1=有錯誤)
pinpoint_status.communication_ok    // 通訊正常 (1=正常, 0=異常)
pinpoint_status.consecutive_failures // 連續失敗次數 (應該是 0)
```

**預期值**：
- `is_ready`: 1
- `has_faults`: 0
- `communication_ok`: 1
- `consecutive_failures`: 0

### 📍 位置資料監控

```
pinpoint_status.position_x_mm       // X 位置 (mm)
pinpoint_status.position_y_mm       // Y 位置 (mm)
pinpoint_status.heading_rad         // 航向 (弧度)
pinpoint_status.heading_deg         // 航向 (度)
```

### 🏃 速度資料監控

```
pinpoint_status.velocity_x_mm_s     // X 速度 (mm/s)
pinpoint_status.velocity_y_mm_s     // Y 速度 (mm/s)
pinpoint_status.velocity_h_rad_s    // 角速度 (rad/s)
```

### 🔍 錯誤診斷監控

```
pinpoint_status.x_pod_detected      // X軸編碼器 (1=偵測到, 0=未偵測到)
pinpoint_status.y_pod_detected      // Y軸編碼器 (1=偵測到, 0=未偵測到)
pinpoint_status.imu_fault           // IMU錯誤 (0=正常, 1=錯誤)
pinpoint_status.bad_read            // 讀取錯誤 (0=正常, 1=錯誤)
```

**預期值**：
- `x_pod_detected`: 1
- `y_pod_detected`: 1
- `imu_fault`: 0
- `bad_read`: 0

### 📊 統計資料監控

```
pinpoint_status.total_updates       // 總更新次數
pinpoint_status.successful_updates  // 成功更新次數
pinpoint_status.last_test_success_rate // 最後測試成功率 (%)
pinpoint_status.last_test_time_ms   // 最後測試時間戳
```

## 🚦 狀態判斷指南

### ✅ 正常狀態指標

- `init_success` = 1
- `communication_ok` = 1
- `is_ready` = 1
- `has_faults` = 0
- `consecutive_failures` = 0
- `device_id` = 48 (0x30)
- `x_pod_detected` = 1
- `y_pod_detected` = 1

### ⚠️ 警告狀態

- `consecutive_failures` > 0 但 < 5
- `last_test_success_rate` < 90% 但 > 70%

### ❌ 錯誤狀態

- `init_success` = 0
- `device_found` = 0
- `communication_ok` = 0
- `consecutive_failures` >= 5
- `device_id` = 0 或 255
- `x_pod_detected` = 0 或 `y_pod_detected` = 0

## 🔧 除錯步驟

### 1. 初始化失敗

如果 `init_success` = 0，檢查：
- `init_step`：失敗在第幾步
- `i2c_ready`：I2C 是否準備好
- `device_found`：是否找到裝置
- `device_id`：裝置 ID 是否正確

### 2. 通訊中斷

如果運行中 `communication_ok` = 0：
- 檢查 `consecutive_failures` 數量
- 查看 `device_status` 錯誤碼
- 確認編碼器連接狀態

### 3. 編碼器問題

如果 `x_pod_detected` 或 `y_pod_detected` = 0：
- 檢查編碼器 pods 連接
- 確認電源供應
- 檢查線路

### 4. 位置異常

如果位置資料不變或異常：
- 確認編碼器是否有移動
- 檢查 IMU 狀態
- 考慮重新校正

## 📝 Live Expressions 設定

1. 在 STM32CubeIDE 中開啟 Live Expressions 視窗
2. 加入上述變數名稱
3. 設定更新頻率為 1-2 秒
4. 開始調試並觀察變數變化

## 💡 使用技巧

- 先確認初始化相關變數，再檢查運行狀態
- 重點監控 `consecutive_failures` 來判斷穩定性
- 定期呼叫 `Chassis::pinpoint_test_communication()` 更新測試結果
- 使用位置和速度資料驗證感測器功能
