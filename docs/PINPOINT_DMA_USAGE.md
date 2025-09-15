# Pinpoint DMA 使用說明

## 概述

Pinpoint 感測器現在支援 DMA (Direct Memory Access) 模式，可以大幅降低 CPU 使用率，提升系統效能。

## 功能特點

### DMA 模式 vs 傳統模式

**傳統模式 (同步):**
- 每次讀取會阻塞 CPU 直到完成
- I2C 通訊期間 CPU 無法處理其他任務
- 適合低頻率讀取場景

**DMA 模式 (異步):**
- 非阻塞式讀取，CPU 可同時處理其他任務
- 硬體自動完成資料傳輸
- 適合高頻率連續讀取
- 可設定讀取間隔，避免過度佔用 I2C 總線

## 使用方式

### 1. 自動初始化 (推薦)

DMA 模式現在會在 `Chassis::setup()` 中自動啟用，無需手動配置：

```cpp
// 在 main 中只需呼叫
Chassis::setup();  // DMA 會自動啟用並配置為 10ms 間隔 (100Hz)
```

### 2. 調整 DMA 配置

如需修改 DMA 設定，可在 `chassis.cpp` 頂部修改常數：

```cpp
// 在 chassis.cpp 中修改這些常數
#define PINPOINT_DMA_ENABLED_DEFAULT    true    // 是否預設啟用 DMA
#define PINPOINT_DMA_INTERVAL_DEFAULT   10      // 預設間隔 (ms)
```

### 3. 運行時控制 (進階)

如果需要在運行時動態調整：

```cpp
// 啟用/停用 DMA
Chassis::pinpoint_enable_dma(true);

// 調整讀取間隔 (5-1000ms)
Chassis::pinpoint_set_dma_interval(5);  // 改為 5ms = 200Hz
```

### 4. 檢查 DMA 狀態

```cpp
bool dma_enabled = Chassis::pinpoint_is_dma_enabled();
```

## 配置參數

### DMA 讀取間隔

- **範圍:** 5ms - 1000ms
- **推薦值:** 10ms (100Hz 更新率)
- **高頻應用:** 5ms (200Hz 更新率)
- **低功耗應用:** 50ms (20Hz 更新率)

```cpp
Chassis::pinpoint_set_dma_interval(10);  // 10ms = 100Hz
```

## 工作原理

### DMA 讀取流程

1. **啟動階段:**
   - 每隔設定的間隔時間，自動啟動 DMA 讀取
   - 使用 Bulk Read 暫存器一次讀取所有資料

2. **傳輸階段:**
   - DMA 硬體自動處理 I2C 通訊
   - CPU 可同時執行其他任務

3. **完成階段:**
   - DMA 完成時觸發中斷
   - 中斷處理函數更新資料狀態
   - 下次 `pinpoint_update()` 呼叫時處理新資料

### 資料內容

DMA 一次讀取包含：
- 位置資料 (X, Y, Heading)
- 速度資料 (X_vel, Y_vel, H_vel)
- 狀態暫存器
- 迴圈時間

## 效能優勢

### CPU 使用率降低

**傳統模式:**
- I2C 讀取時 CPU 等待: ~1-2ms
- 多次讀取累積等待時間較長

**DMA 模式:**
- CPU 等待時間: ~0.1ms (僅處理資料)
- 95% 的傳輸時間 CPU 可處理其他任務

### 實時性改善

- 更穩定的讀取間隔
- 減少因 CPU 負載變化導致的時序偏差
- 更適合即時控制應用

## 錯誤處理

### 自動降級機制

如果 DMA 操作失敗，系統會自動降級到傳統同步模式：

```cpp
// DMA 失敗時的 fallback 處理
if (dma_start_failed) {
    pinpoint_status.communication_ok = 0;
    pinpoint_status.consecutive_failures++;
    // 自動切換到同步模式繼續運行
}
```

### 狀態監控

可透過 `pinpoint_status` 結構監控 DMA 狀態：

```cpp
// 檢查通訊狀態
if (pinpoint_status.communication_ok == 0) {
    // 處理通訊錯誤
}

// 檢查連續失敗次數
if (pinpoint_status.consecutive_failures > 5) {
    // 考慮重新初始化
}
```

## 最佳實踐

### 1. 選擇適當的讀取間隔

```cpp
// 高精度控制 (如平衡機器人)
Chassis::pinpoint_set_dma_interval(5);   // 200Hz

// 一般導航應用
Chassis::pinpoint_set_dma_interval(10);  // 100Hz

// 低功耗應用
Chassis::pinpoint_set_dma_interval(50);  // 20Hz
```

### 2. 監控系統效能

```cpp
// 定期檢查更新成功率
float success_rate = (float)pinpoint_status.successful_updates / 
                     pinpoint_status.total_updates * 100.0f;

if (success_rate < 90.0f) {
    // 考慮增加讀取間隔或檢查硬體
}
```

### 3. I2C 總線配置

確保 I2C DMA 配置正確：

- I2C2 必須配置 DMA 通道
- DMA 中斷優先級設定適當
- `HAL_I2C_MasterRxCpltCallback` 已正確實作

## 故障排除

### 常見問題

1. **DMA 無法啟動**
   - 檢查 I2C DMA 配置
   - 確認 DMA 通道沒有衝突

2. **資料更新不穩定**
   - 調整讀取間隔
   - 檢查 I2C 總線品質

3. **CPU 使用率仍然很高**
   - 確認其他任務沒有佔用過多時間
   - 檢查中斷處理函數效率

### 除錯資訊

使用 Live Expressions 監控：

```cpp
// DMA 狀態
pinpoint.dma_state
pinpoint.dma_read_complete

// 效能指標
pinpoint_status.successful_updates
pinpoint_status.total_updates
pinpoint_status.consecutive_failures
```

## 版本歷史

- v2.0: 新增 DMA 支援
- v1.0: 基本同步模式

## 注意事項

1. DMA 模式需要正確的硬體配置
2. 在除錯時可能需要暫時停用 DMA 以簡化問題
3. 確保 I2C 總線上沒有其他高頻率裝置干擾
