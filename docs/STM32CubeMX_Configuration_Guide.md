# STM32CubeMX 配置指南 - TDK2025_chassis專案

## 概述
本指南說明如何在STM32CubeMX中配置STM32H723ZG微控制器，以支援：
- goBILDA Pinpoint OTOS感測器（I2C通信）
- ROS串行通信（UART）
- 底盤馬達控制（PWM/Timer）
- VL53L0X感測器（I2C）
- 升降機控制系統

---

## 1. 基本系統配置

### 1.1 時鐘配置 (Clock Configuration)

```
HSE: 25 MHz (外部晶振)
SYSCLK: 550 MHz (最大時鐘)
HCLK: 275 MHz
APB1: 137.5 MHz  
APB2: 137.5 MHz
APB3: 137.5 MHz
APB4: 137.5 MHz
```

**配置步驟**：
1. 進入 **Clock Configuration** 標籤
2. 設定 **HSE** 為 25MHz
3. 啟用 **PLL1**，設定倍頻以達到最大時鐘頻率
4. 確保所有外設時鐘正確分頻

---

## 2. 通信接口配置

### 2.1 I2C配置 - Pinpoint OTOS和VL53L0X感測器

#### I2C1 配置（Pinpoint OTOS）
```
模式: I2C
I2C Speed Mode: Fast Mode (400 kHz)
Clock No Stretch Mode: Disabled
General Call Address Detection: Disabled
```

**引腳配置**：
- **SCL**: PB8 (I2C1_SCL)
- **SDA**: PB9 (I2C1_SDA)

**詳細參數**：
```
I2C Clock Speed: 400000 Hz
Rise Time: 120 ns
Fall Time: 25 ns
Coefficient of Digital Filter: 0
Analog Filter: Enable
```

#### I2C2配置（VL53L0X感測器）
```
模式: I2C  
I2C Speed Mode: Fast Mode (400 kHz)
```

**引腳配置**：
- **SCL**: PF1 (I2C2_SCL)
- **SDA**: PF0 (I2C2_SDA)

#### DMA配置（提升I2C性能）
```
I2C1_RX: DMA1 Stream 0, Channel 1
I2C1_TX: DMA1 Stream 1, Channel 1
I2C2_RX: DMA1 Stream 2, Channel 3
I2C2_TX: DMA1 Stream 3, Channel 3
```

### 2.2 UART配置 - ROS通信

#### USART3配置（ROS串行通信）
```
模式: Asynchronous
Baud Rate: 115200
Word Length: 8 Bits
Parity: None
Stop Bits: 1
Hardware Flow Control: None
```

**引腳配置**：
- **TX**: PD8 (USART3_TX)
- **RX**: PD9 (USART3_RX)

**DMA配置**：
```
USART3_RX: DMA1 Stream 4, Channel 4
USART3_TX: DMA1 Stream 5, Channel 4
```

**中斷優先級**：
```
USART3 global interrupt: Priority 5, Sub Priority 0
DMA1 Stream4 interrupt: Priority 6, Sub Priority 0
DMA1 Stream5 interrupt: Priority 6, Sub Priority 0
```

---

## 3. 馬達控制配置

### 3.1 PWM Timer配置

#### TIM1 - 主要馬達PWM
```
模式: PWM Generation CH1,CH2,CH3,CH4
Prescaler: 275-1 (1MHz計數頻率)
Counter Period: 1000-1 (1kHz PWM頻率)
Pulse: 0 (初始占空比0%)
```

**引腳配置**：
- **CH1**: PE9 (TIM1_CH1) - 前左馬達
- **CH2**: PE11 (TIM1_CH2) - 前右馬達  
- **CH3**: PE13 (TIM1_CH3) - 後左馬達
- **CH4**: PE14 (TIM1_CH4) - 後右馬達

#### TIM4 - 升降機馬達PWM
```
模式: PWM Generation CH1,CH2
Prescaler: 275-1
Counter Period: 1000-1
```

**引腳配置**：
- **CH1**: PD12 (TIM4_CH1) - 前升降機
- **CH2**: PD13 (TIM4_CH2) - 後升降機

### 3.2 編碼器Timer配置

#### TIM2 - 編碼器接口
```
模式: Encoder Mode
Encoder Mode: TI1 and TI2 as inputs
Counter Period: 65535
```

**引腳配置**：
- **CH1**: PA15 (TIM2_CH1) - 編碼器A
- **CH2**: PB3 (TIM2_CH2) - 編碼器B

#### TIM3 - 第二編碼器
```
模式: Encoder Mode
```

**引腳配置**：
- **CH1**: PC6 (TIM3_CH1)
- **CH2**: PC7 (TIM3_CH2)

---

## 4. GPIO配置

### 4.1 馬達方向控制
```
PA0: GPIO_Output (Motor1_DIR)
PA1: GPIO_Output (Motor2_DIR)  
PA2: GPIO_Output (Motor3_DIR)
PA3: GPIO_Output (Motor4_DIR)
```

**配置參數**：
```
Output Level: Low
Mode: Output Push Pull
Pull-up/Pull-down: No pull-up and no pull-down
Maximum output speed: High
```

### 4.2 VL53L0X感測器控制
```
PC0: GPIO_Output (VL53_XSHUT1)
PC1: GPIO_Output (VL53_XSHUT2)
PC2: GPIO_Input (VL53_INT1)
PC3: GPIO_Input (VL53_INT2)
```

### 4.3 狀態LED
```
PB0: GPIO_Output (LED_RED)
PB1: GPIO_Output (LED_GREEN)
PB14: GPIO_Output (LED_BLUE)
```

---

## 5. 系統Timer配置

### 5.1 主要系統Timer

#### TIM6 - 底盤控制Timer
```
模式: Internal Clock
Prescaler: 27500-1 (10kHz)
Counter Period: 100-1 (100Hz = 10ms週期)
啟用中斷: TIM6 global interrupt
```

#### TIM7 - ROS發布Timer  
```
模式: Internal Clock
Prescaler: 27500-1 (10kHz)
Counter Period: 500-1 (20Hz = 50ms週期)
啟用中斷: TIM7 global interrupt
```

#### TIM13 - 感測器更新Timer
```
模式: Internal Clock  
Prescaler: 27500-1 (10kHz)
Counter Period: 200-1 (50Hz = 20ms週期)
啟用中斷: TIM13 global interrupt
```

---

## 6. 中斷優先級配置

### 6.1 NVIC設定
```
TIM6 global interrupt: Priority 3, Sub Priority 0 (底盤控制-高優先級)
TIM7 global interrupt: Priority 4, Sub Priority 0 (ROS發布)
TIM13 global interrupt: Priority 4, Sub Priority 1 (感測器更新)
USART3 global interrupt: Priority 5, Sub Priority 0 (ROS通信)
I2C1 event interrupt: Priority 6, Sub Priority 0 (Pinpoint通信)
I2C2 event interrupt: Priority 6, Sub Priority 1 (VL53通信)
```

---

## 7. DMA配置摘要

### 7.1 DMA1配置
```
Stream 0: I2C1_RX (Channel 1)
Stream 1: I2C1_TX (Channel 1)  
Stream 2: I2C2_RX (Channel 3)
Stream 3: I2C2_TX (Channel 3)
Stream 4: USART3_RX (Channel 4)
Stream 5: USART3_TX (Channel 4)
```

### 7.2 DMA設定參數
```
Mode: Normal (除了USART可用Circular)
Priority: Medium
Data Width: Byte
Increment Address: Memory increment
```

---

## 8. 電源管理配置

### 8.1 PWR設定
```
Power Regulator Voltage Scale: Scale 0 (高性能)
Over Drive: Enable
```

### 8.2 低功耗模式
```
Low Power Mode: None (保持全功率運行)
```

---

## 9. 調試配置

### 9.1 SYS配置
```
Debug: Serial Wire (SWD)
Timebase Source: TIM15 (避免與HAL_Delay衝突)
```

---

## 10. 生成代碼設定

### 10.1 Project Settings
```
Project Name: TDK2025_chassis
Toolchain/IDE: STM32CubeIDE
Firmware Package: Latest Version
Do not generate the main(): 取消勾選 (生成main.c)
```

### 10.2 Code Generator
```
STM32Cube MCU packages and embedded software packs: Add necessary libraries
Generated files: 
  ☑ Generate peripheral initialization as a pair of '.c/.h' files per peripheral
  ☑ Set all free pins as analog
  ☑ Initialize all peripherals in their default Mode
```

---

## 11. 配置後檢查清單

### 11.1 必須確認的項目
- [ ] I2C時鐘頻率設定為400kHz
- [ ] UART波特率設定為115200
- [ ] PWM頻率設定為1kHz
- [ ] 所有中斷優先級正確設定
- [ ] DMA配置無衝突
- [ ] GPIO引腳分配正確
- [ ] 時鐘樹配置達到最大性能

### 11.2 生成代碼後的驗證
```c
// 在main.c中驗證配置
printf("SYSCLK: %lu Hz\n", HAL_RCC_GetSysClockFreq());
printf("HCLK: %lu Hz\n", HAL_RCC_GetHCLKFreq());
printf("PCLK1: %lu Hz\n", HAL_RCC_GetPCLK1Freq());
printf("PCLK2: %lu Hz\n", HAL_RCC_GetPCLK2Freq());
```

---

## 12. 與現有代碼整合

### 12.1 在main.c中添加初始化
```c
// 在main函數中，MX_GPIO_Init()後添加
Pinpoint_Chassis_Init();
ROS1::init();
Chassis::setup();
```

### 12.2 在中斷回調中添加處理
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        // 底盤控制更新 (100Hz)
        Chassis::updateSpeed(cmd_v_x, cmd_v_y, cmd_v_w);
    }
    else if (htim->Instance == TIM7) {
        // ROS發布更新 (20Hz)
        ROS1::pub_chassis_pose();
    }
    else if (htim->Instance == TIM13) {
        // 感測器更新 (50Hz)
        Pinpoint_Chassis_Update();
        ROS1::spinCycle();
    }
}
```

這個配置將為您的TDK2025_chassis專案提供完整的硬體支援，包括高精度的Pinpoint感測器整合！
