# STM32CubeMX 配置步驟圖解

## 📋 配置順序檢查清單

### ✅ 第一步：基本系統配置
1. **打開STM32CubeMX**
2. **選擇MCU**: STM32H723ZGTx
3. **配置時鐘樹**：
   ```
   STM32CubeMX → Clock Configuration Tab
   HSE: 25MHz → PLL1 → SYSCLK: 550MHz
   ```

### ✅ 第二步：引腳配置 (Pinout & Configuration)

#### 🔌 I2C配置
```
左側面板 → Connectivity → I2C1
Mode: I2C ✓
Configuration Tab:
├─ Parameter Settings:
│  ├─ I2C Speed Mode: Fast Mode
│  └─ I2C Clock Speed: 400000
├─ GPIO Settings:
│  ├─ PB8: I2C1_SCL
│  └─ PB9: I2C1_SDA
└─ DMA Settings:
   ├─ Add → I2C1_RX → DMA1 Stream0 Channel1
   └─ Add → I2C1_TX → DMA1 Stream1 Channel1
```

#### 📡 UART配置 (ROS通信)
```
左側面板 → Connectivity → USART3
Mode: Asynchronous ✓
Configuration Tab:
├─ Parameter Settings:
│  ├─ Baud Rate: 115200
│  ├─ Word Length: 8 Bits
│  ├─ Parity: None
│  └─ Stop Bits: 1
├─ GPIO Settings:
│  ├─ PD8: USART3_TX
│  └─ PD9: USART3_RX
└─ DMA Settings:
   ├─ Add → USART3_RX → DMA1 Stream4 Channel4
   └─ Add → USART3_TX → DMA1 Stream5 Channel4
```

#### ⚡ PWM配置 (馬達控制)
```
左側面板 → Timers → TIM1
Mode: Internal Clock ✓
Channels:
├─ Channel1: PWM Generation CH1 ✓
├─ Channel2: PWM Generation CH2 ✓
├─ Channel3: PWM Generation CH3 ✓
└─ Channel4: PWM Generation CH4 ✓

Configuration Tab:
├─ Parameter Settings:
│  ├─ Prescaler: 274 (275-1)
│  ├─ Counter Period: 999 (1000-1)
│  └─ auto-reload preload: Enable
└─ GPIO Settings:
   ├─ PE9: TIM1_CH1 (前左馬達)
   ├─ PE11: TIM1_CH2 (前右馬達)
   ├─ PE13: TIM1_CH3 (後左馬達)
   └─ PE14: TIM1_CH4 (後右馬達)
```

### ✅ 第三步：Timer中斷配置

#### ⏰ 系統Timer設定
```
TIM6 (底盤控制 - 100Hz):
├─ Mode: Internal Clock ✓
├─ Prescaler: 27499 (27500-1)
├─ Counter Period: 99 (100-1)
└─ NVIC Settings: ✓ TIM6 global interrupt

TIM7 (ROS發布 - 20Hz):  
├─ Mode: Internal Clock ✓
├─ Prescaler: 27499 (27500-1)
├─ Counter Period: 499 (500-1)
└─ NVIC Settings: ✓ TIM7 global interrupt

TIM13 (感測器更新 - 50Hz):
├─ Mode: Internal Clock ✓
├─ Prescaler: 27499 (27500-1)
├─ Counter Period: 199 (200-1)
└─ NVIC Settings: ✓ TIM13 global interrupt
```

### ✅ 第四步：GPIO配置

#### 🔧 馬達方向控制
```
在Pinout view中點擊引腳並設定：
PA0 → GPIO_Output (Motor1_DIR)
PA1 → GPIO_Output (Motor2_DIR)
PA2 → GPIO_Output (Motor3_DIR)
PA3 → GPIO_Output (Motor4_DIR)

右鍵 → GPIO Configuration:
├─ GPIO output level: Low
├─ GPIO mode: Output Push Pull
├─ GPIO Pull-up/Pull-down: No pull-up and no pull-down
└─ Maximum output speed: High
```

#### 🌐 VL53L0X感測器控制
```
PC0 → GPIO_Output (VL53_XSHUT1)
PC1 → GPIO_Output (VL53_XSHUT2)
PC2 → GPIO_Input (VL53_INT1)
PC3 → GPIO_Input (VL53_INT2)
```

### ✅ 第五步：中斷優先級設定

```
左側面板 → System Core → NVIC
Configuration Tab → NVIC → Code generation:

優先級設定 (數字越小優先級越高):
├─ TIM6 global interrupt: Preemption=3, Sub=0
├─ TIM7 global interrupt: Preemption=4, Sub=0  
├─ TIM13 global interrupt: Preemption=4, Sub=1
├─ USART3 global interrupt: Preemption=5, Sub=0
├─ I2C1 event interrupt: Preemption=6, Sub=0
└─ I2C2 event interrupt: Preemption=6, Sub=1
```

### ✅ 第六步：DMA檢查

```
左側面板 → System Core → DMA
檢查DMA請求沒有衝突:

DMA1:
├─ Stream 0: I2C1_RX
├─ Stream 1: I2C1_TX
├─ Stream 2: I2C2_RX (如果使用I2C2)
├─ Stream 3: I2C2_TX (如果使用I2C2)
├─ Stream 4: USART3_RX
└─ Stream 5: USART3_TX
```

---

## 🚨 常見配置錯誤及解決方法

### ❌ 錯誤1：I2C無法工作
**問題**：I2C通信失敗或設備無回應
**解決**：
- 檢查SCL/SDA引腳是否正確配置
- 確認I2C時鐘頻率設為400kHz
- 檢查上拉電阻（外部3.3kΩ）

### ❌ 錯誤2：UART通信不穩定  
**問題**：ROS通信有checksum錯誤
**解決**：
- 確認波特率115200
- 啟用DMA for USART3
- 檢查引腳配置：PD8(TX), PD9(RX)

### ❌ 錯誤3：PWM頻率不正確
**問題**：馬達運行不順或有噪音
**解決**：
- Prescaler計算：SYSCLK/PWM_freq/1000
- 對於550MHz系統：Prescaler = 550MHz/1MHz = 550-1 = 549
- Counter Period = 1000-1 = 999 (1kHz PWM)

### ❌ 錯誤4：Timer中斷衝突
**問題**：系統運行不穩定
**解決**：
- 檢查中斷優先級設定
- 確保高頻中斷有較高優先級
- 避免在中斷中執行長時間操作

---

## 📱 CubeMX操作截圖指南

### 1️⃣ 主界面操作
```
File → New Project → Board Selector 
搜索：STM32H723ZG
選擇：NUCLEO-H723ZG 或 Generic STM32H723ZGTx
```

### 2️⃣ 引腳配置界面
```
中央Pinout view → 點擊引腳 → 選擇功能
或
左側Pinout & Configuration → 展開分類 → 勾選功能
```

### 3️⃣ 時鐘配置界面
```
Clock Configuration Tab → 
調整輸入時鐘 → 設定PLL → 檢查輸出時鐘
```

### 4️⃣ 代碼生成
```
Project Manager Tab → Project Settings:
├─ Project Name: TDK2025_chassis
├─ Project Location: 選擇workspace路徑  
├─ Toolchain/IDE: STM32CubeIDE
└─ 點擊 "GENERATE CODE"
```

---

## 🔧 生成代碼後的驗證

### 在main.c中添加測試代碼：
```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM13_Init();
    
    // 驗證配置
    printf("System Clock: %lu Hz\n", HAL_RCC_GetSysClockFreq());
    printf("Configuration completed!\n");
    
    // 啟動Timer
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim13);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    while (1) {
        // 主循環
    }
}
```

**按照這個指南配置後，您的STM32H723ZG將完全支援TDK2025_chassis專案的所有功能！** 🎯
