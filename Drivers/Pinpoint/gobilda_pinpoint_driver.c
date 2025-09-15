/*
 * gobilda_pinpoint_driver.c
 *
 * STM32 HAL Driver for goBILDA Pinpoint OTOS (Optical Tracking Odometry Sensor)
 * Based on Product 3110-0002-0001 User Guide
 *
 * Created on: Sep 1, 2025
 * Author: Your Name
 */

#include "gobilda_pinpoint_driver.h"
#include <string.h>
#include <math.h>

/* Private Constants */
#define PINPOINT_I2C_TIMEOUT        1000
#define GOBILDA_SWINGARM_POD_TICKS   13.26291192f  // ticks per mm
#define GOBILDA_4_BAR_POD_TICKS      19.89436789f  // ticks per mm

/* Unit conversion constants */
#define MM_TO_CM                     0.1f
#define MM_TO_M                      0.001f
#define MM_TO_INCH                   0.0393701f
#define RAD_TO_DEG                   57.2957795f
#define DEG_TO_RAD                   0.0174532925f

/* Private Function Prototypes */
static HAL_StatusTypeDef pinpoint_write_int(pinpoint_handle_t *handle, pinpoint_register_t reg, int32_t value);
static HAL_StatusTypeDef pinpoint_read_int(pinpoint_handle_t *handle, pinpoint_register_t reg, int32_t *value);
static HAL_StatusTypeDef pinpoint_write_float(pinpoint_handle_t *handle, pinpoint_register_t reg, float value);
static HAL_StatusTypeDef pinpoint_read_float(pinpoint_handle_t *handle, pinpoint_register_t reg, float *value);
static HAL_StatusTypeDef pinpoint_write_byte(pinpoint_handle_t *handle, pinpoint_register_t reg, uint8_t value);
static HAL_StatusTypeDef pinpoint_read_byte(pinpoint_handle_t *handle, pinpoint_register_t reg, uint8_t *value);

/* Private Helper Functions */

/**
 * @brief Write a 32-bit integer to device register
 */
static HAL_StatusTypeDef pinpoint_write_int(pinpoint_handle_t *handle, pinpoint_register_t reg, int32_t value)
{
    uint8_t data[5];
    data[0] = (uint8_t)reg;
    data[1] = (uint8_t)(value & 0xFF);
    data[2] = (uint8_t)((value >> 8) & 0xFF);
    data[3] = (uint8_t)((value >> 16) & 0xFF);
    data[4] = (uint8_t)((value >> 24) & 0xFF);
    
    return HAL_I2C_Master_Transmit(handle->hi2c, handle->device_address << 1, data, 5, PINPOINT_I2C_TIMEOUT);
}

/**
 * @brief Read a 32-bit integer from device register
 */
static HAL_StatusTypeDef pinpoint_read_int(pinpoint_handle_t *handle, pinpoint_register_t reg, int32_t *value)
{
    uint8_t reg_addr = (uint8_t)reg;
    uint8_t data[4];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(handle->hi2c, handle->device_address << 1, &reg_addr, 1, PINPOINT_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    
    status = HAL_I2C_Master_Receive(handle->hi2c, handle->device_address << 1, data, 4, PINPOINT_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    
    *value = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    return HAL_OK;
}

/**
 * @brief Write a float to device register
 */
static HAL_StatusTypeDef pinpoint_write_float(pinpoint_handle_t *handle, pinpoint_register_t reg, float value)
{
    uint8_t data[5];
    uint32_t *float_as_int = (uint32_t*)&value;
    
    data[0] = (uint8_t)reg;
    data[1] = (uint8_t)(*float_as_int & 0xFF);
    data[2] = (uint8_t)((*float_as_int >> 8) & 0xFF);
    data[3] = (uint8_t)((*float_as_int >> 16) & 0xFF);
    data[4] = (uint8_t)((*float_as_int >> 24) & 0xFF);
    
    return HAL_I2C_Master_Transmit(handle->hi2c, handle->device_address << 1, data, 5, PINPOINT_I2C_TIMEOUT);
}

/**
 * @brief Read a float from device register
 */
static HAL_StatusTypeDef pinpoint_read_float(pinpoint_handle_t *handle, pinpoint_register_t reg, float *value)
{
    uint8_t reg_addr = (uint8_t)reg;
    uint8_t data[4];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(handle->hi2c, handle->device_address << 1, &reg_addr, 1, PINPOINT_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    
    status = HAL_I2C_Master_Receive(handle->hi2c, handle->device_address << 1, data, 4, PINPOINT_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    
    uint32_t int_val = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    *value = *(float*)&int_val;
    return HAL_OK;
}

/**
 * @brief Write a byte to device register
 */
static HAL_StatusTypeDef pinpoint_write_byte(pinpoint_handle_t *handle, pinpoint_register_t reg, uint8_t value)
{
    uint8_t data[2] = {(uint8_t)reg, value};
    return HAL_I2C_Master_Transmit(handle->hi2c, handle->device_address << 1, data, 2, PINPOINT_I2C_TIMEOUT);
}

/**
 * @brief Read a byte from device register
 */
static HAL_StatusTypeDef pinpoint_read_byte(pinpoint_handle_t *handle, pinpoint_register_t reg, uint8_t *value)
{
    uint8_t reg_addr = (uint8_t)reg;
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(handle->hi2c, handle->device_address << 1, &reg_addr, 1, PINPOINT_I2C_TIMEOUT);
    if (status != HAL_OK) return status;
    
    return HAL_I2C_Master_Receive(handle->hi2c, handle->device_address << 1, value, 1, PINPOINT_I2C_TIMEOUT);
}

/* Public Functions Implementation */

/**
 * @brief Initialize the Pinpoint driver
 */
HAL_StatusTypeDef Pinpoint_Init(pinpoint_handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t device_addr)
{
    if (handle == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }
    
    // Initialize handle structure
    memset(handle, 0, sizeof(pinpoint_handle_t));
    handle->hi2c = hi2c;
    handle->device_address = device_addr;
    
    // Set default units
    handle->distance_unit = PINPOINT_UNIT_MM;
    handle->angle_unit = PINPOINT_ANGLE_RADIANS;
    
    // Set default yaw scalar
    handle->yaw_scalar = 1.0f;
    
    // Wait for device to be ready
    HAL_Delay(100);
    
    // Check device status
    HAL_StatusTypeDef status = Pinpoint_GetDeviceStatus(handle, &handle->device_status);
    return status;
}

/**
 * @brief Reset the Pinpoint device
 */
HAL_StatusTypeDef Pinpoint_Reset(pinpoint_handle_t *handle)
{
    if (handle == NULL) return HAL_ERROR;
    
    // Send reset command (writing 1 to device control register typically triggers reset)
    HAL_StatusTypeDef status = pinpoint_write_byte(handle, PINPOINT_REG_DEVICE_CONTROL, 1);
    
    if (status == HAL_OK) {
        HAL_Delay(500); // Wait for reset to complete
        // Clear local data
        memset(&handle->current_pose, 0, sizeof(pinpoint_pose_t));
        memset(&handle->current_velocity, 0, sizeof(pinpoint_velocity_t));
        handle->x_encoder_value = 0;
        handle->y_encoder_value = 0;
    }
    
    return status;
}

/**
 * @brief Set encoder pod offsets
 */
HAL_StatusTypeDef Pinpoint_SetOffsets(pinpoint_handle_t *handle, float x_offset, float y_offset)
{
    if (handle == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status;
    
    status = pinpoint_write_float(handle, PINPOINT_REG_X_POD_OFFSET, x_offset);
    if (status != HAL_OK) return status;
    
    status = pinpoint_write_float(handle, PINPOINT_REG_Y_POD_OFFSET, y_offset);
    if (status != HAL_OK) return status;
    
    // Update local values
    handle->x_pod_offset = x_offset;
    handle->y_pod_offset = y_offset;
    
    return HAL_OK;
}

/**
 * @brief Set encoder resolution using goBILDA pod type
 */
HAL_StatusTypeDef Pinpoint_SetEncoderResolution_goBILDA(pinpoint_handle_t *handle, gobilda_pod_type_t pod_type)
{
    if (handle == NULL) return HAL_ERROR;
    
    float mm_per_tick;
    
    switch (pod_type) {
        case GOBILDA_SWINGARM_POD:
            mm_per_tick = 1.0f / GOBILDA_SWINGARM_POD_TICKS;
            break;
        case GOBILDA_4_BAR_POD:
            mm_per_tick = 1.0f / GOBILDA_4_BAR_POD_TICKS;
            break;
        default:
            return HAL_ERROR;
    }
    
    return Pinpoint_SetEncoderResolution(handle, mm_per_tick);
}

/**
 * @brief Set encoder resolution manually
 */
HAL_StatusTypeDef Pinpoint_SetEncoderResolution(pinpoint_handle_t *handle, float mm_per_tick)
{
    if (handle == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status = pinpoint_write_float(handle, PINPOINT_REG_MM_PER_TICK, mm_per_tick);
    if (status == HAL_OK) {
        handle->mm_per_tick = mm_per_tick;
    }
    
    return status;
}

/**
 * @brief Set encoder directions
 */
HAL_StatusTypeDef Pinpoint_SetEncoderDirections(pinpoint_handle_t *handle, 
                                                pinpoint_encoder_direction_t x_direction,
                                                pinpoint_encoder_direction_t y_direction)
{
    if (handle == NULL) return HAL_ERROR;
    
    // Combine directions into a single byte: bit 0 = X direction, bit 1 = Y direction
    uint8_t direction_byte = (x_direction & 0x01) | ((y_direction & 0x01) << 1);
    
    // Note: The actual register for encoder directions may differ in the real device
    // This is an assumption based on typical implementations
    return pinpoint_write_byte(handle, PINPOINT_REG_DEVICE_CONTROL, direction_byte);
}

/**
 * @brief Set yaw scalar for heading correction
 */
HAL_StatusTypeDef Pinpoint_SetYawScalar(pinpoint_handle_t *handle, float yaw_scalar)
{
    if (handle == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status = pinpoint_write_float(handle, PINPOINT_REG_YAW_SCALAR, yaw_scalar);
    if (status == HAL_OK) {
        handle->yaw_scalar = yaw_scalar;
    }
    
    return status;
}

/**
 * @brief Update all sensor readings
 */
HAL_StatusTypeDef Pinpoint_Update(pinpoint_handle_t *handle)
{
    if (handle == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status;
    
    // Read device status
    status = pinpoint_read_byte(handle, PINPOINT_REG_DEVICE_STATUS, &handle->device_status);
    if (status != HAL_OK) return status;
    
    // Read encoder values
    status = pinpoint_read_int(handle, PINPOINT_REG_X_ENCODER_VALUE, &handle->x_encoder_value);
    if (status != HAL_OK) return status;
    
    status = pinpoint_read_int(handle, PINPOINT_REG_Y_ENCODER_VALUE, &handle->y_encoder_value);
    if (status != HAL_OK) return status;
    
    // Read position
    status = pinpoint_read_float(handle, PINPOINT_REG_X_POSITION, &handle->current_pose.x);
    if (status != HAL_OK) return status;
    
    status = pinpoint_read_float(handle, PINPOINT_REG_Y_POSITION, &handle->current_pose.y);
    if (status != HAL_OK) return status;
    
    status = pinpoint_read_float(handle, PINPOINT_REG_H_ORIENTATION, &handle->current_pose.heading);
    if (status != HAL_OK) return status;
    
    // Read velocity
    status = pinpoint_read_float(handle, PINPOINT_REG_X_VELOCITY, &handle->current_velocity.x_velocity);
    if (status != HAL_OK) return status;
    
    status = pinpoint_read_float(handle, PINPOINT_REG_Y_VELOCITY, &handle->current_velocity.y_velocity);
    if (status != HAL_OK) return status;
    
    status = pinpoint_read_float(handle, PINPOINT_REG_H_VELOCITY, &handle->current_velocity.h_velocity);
    if (status != HAL_OK) return status;
    
    // Read loop time
    int32_t loop_time_int;
    status = pinpoint_read_int(handle, PINPOINT_REG_LOOP_TIME, &loop_time_int);
    if (status == HAL_OK) {
        handle->loop_time = (uint32_t)loop_time_int;
    }
    
    // 應用軸向修正以匹配實際機械配置（與 DMA 模式保持一致）
    // 交換 X/Y 軸並修正 X 軸正負號
    int32_t temp_x_encoder = handle->x_encoder_value;
    int32_t temp_y_encoder = handle->y_encoder_value;
    float temp_x_pos = handle->current_pose.x;
    float temp_y_pos = handle->current_pose.y;
    float temp_x_vel = handle->current_velocity.x_velocity;
    float temp_y_vel = handle->current_velocity.y_velocity;
    
    // 軸向修正：X 軸反向，Y 軸直接對應
    handle->x_encoder_value = -temp_x_encoder;  // X 軸 = -X 寄存器（反向）
    handle->y_encoder_value = temp_y_encoder;   // Y 軸 = Y 寄存器（直接對應）
    handle->current_pose.x = -temp_x_pos;       // X 位置 = -X 寄存器（反向）
    handle->current_pose.y = temp_y_pos;        // Y 位置 = Y 寄存器（直接對應）
    handle->current_velocity.x_velocity = -temp_x_vel;  // X 速度 = -X 寄存器（反向）
    handle->current_velocity.y_velocity = temp_y_vel;   // Y 速度 = Y 寄存器（直接對應）
    
    return status;
}

/**
 * @brief Recalibrate the IMU
 */
HAL_StatusTypeDef Pinpoint_RecalibrateIMU(pinpoint_handle_t *handle)
{
    if (handle == NULL) return HAL_ERROR;
    
    // Send calibration command (specific command may vary)
    return pinpoint_write_byte(handle, PINPOINT_REG_DEVICE_CONTROL, 2);
}

/**
 * @brief Reset position to (0, 0, 0)
 */
HAL_StatusTypeDef Pinpoint_ResetPosition(pinpoint_handle_t *handle)
{
    return Pinpoint_SetPosition(handle, 0.0f, 0.0f, 0.0f);
}

/**
 * @brief Set position manually
 */
HAL_StatusTypeDef Pinpoint_SetPosition(pinpoint_handle_t *handle, float x, float y, float heading)
{
    if (handle == NULL) return HAL_ERROR;
    
    HAL_StatusTypeDef status;
    
    status = pinpoint_write_float(handle, PINPOINT_REG_X_POSITION, x);
    if (status != HAL_OK) return status;
    
    status = pinpoint_write_float(handle, PINPOINT_REG_Y_POSITION, y);
    if (status != HAL_OK) return status;
    
    status = pinpoint_write_float(handle, PINPOINT_REG_H_ORIENTATION, heading);
    if (status != HAL_OK) return status;
    
    // Update local values
    handle->current_pose.x = x;
    handle->current_pose.y = y;
    handle->current_pose.heading = heading;
    
    return HAL_OK;
}

/**
 * @brief Get current position
 */
HAL_StatusTypeDef Pinpoint_GetPosition(pinpoint_handle_t *handle, pinpoint_pose_t *pose)
{
    if (handle == NULL || pose == NULL) return HAL_ERROR;
    
    *pose = handle->current_pose;
    return HAL_OK;
}

/**
 * @brief Get current velocity
 */
HAL_StatusTypeDef Pinpoint_GetVelocity(pinpoint_handle_t *handle, pinpoint_velocity_t *velocity)
{
    if (handle == NULL || velocity == NULL) return HAL_ERROR;
    
    *velocity = handle->current_velocity;
    return HAL_OK;
}

/**
 * @brief Get device status
 */
HAL_StatusTypeDef Pinpoint_GetDeviceStatus(pinpoint_handle_t *handle, uint8_t *status)
{
    if (handle == NULL || status == NULL) return HAL_ERROR;
    
    return pinpoint_read_byte(handle, PINPOINT_REG_DEVICE_STATUS, status);
}

/**
 * @brief Check if device is ready
 */
bool Pinpoint_IsReady(pinpoint_handle_t *handle)
{
    if (handle == NULL) return false;
    
    return (handle->device_status & PINPOINT_STATUS_READY) != 0;
}

/**
 * @brief Check if device has any faults
 */
bool Pinpoint_HasFaults(pinpoint_handle_t *handle)
{
    if (handle == NULL) return true;
    
    return (handle->device_status & (PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED | 
                                    PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED |
                                    PINPOINT_STATUS_FAULT_IMU_RUNAWAY |
                                    PINPOINT_STATUS_FAULT_BAD_READ)) != 0;
}

/**
 * @brief Get device ID
 */
HAL_StatusTypeDef Pinpoint_GetDeviceID(pinpoint_handle_t *handle, uint8_t *device_id)
{
    if (handle == NULL || device_id == NULL) return HAL_ERROR;
    
    return pinpoint_read_byte(handle, PINPOINT_REG_DEVICE_ID, device_id);
}

/**
 * @brief Get device version
 */
HAL_StatusTypeDef Pinpoint_GetDeviceVersion(pinpoint_handle_t *handle, uint8_t *version)
{
    if (handle == NULL || version == NULL) return HAL_ERROR;
    
    return pinpoint_read_byte(handle, PINPOINT_REG_DEVICE_VERSION, version);
}

/**
 * @brief Convert distance units
 */
float Pinpoint_ConvertDistance(float value, pinpoint_distance_unit_t from_unit, pinpoint_distance_unit_t to_unit)
{
    if (from_unit == to_unit) return value;
    
    // Convert to mm first
    float mm_value = value;
    switch (from_unit) {
        case PINPOINT_UNIT_CM:
            mm_value = value * 10.0f;
            break;
        case PINPOINT_UNIT_M:
            mm_value = value * 1000.0f;
            break;
        case PINPOINT_UNIT_INCH:
            mm_value = value / MM_TO_INCH;
            break;
        case PINPOINT_UNIT_MM:
        default:
            break;
    }
    
    // Convert from mm to target unit
    switch (to_unit) {
        case PINPOINT_UNIT_CM:
            return mm_value * MM_TO_CM;
        case PINPOINT_UNIT_M:
            return mm_value * MM_TO_M;
        case PINPOINT_UNIT_INCH:
            return mm_value * MM_TO_INCH;
        case PINPOINT_UNIT_MM:
        default:
            return mm_value;
    }
}

/**
 * @brief Convert angle units
 */
float Pinpoint_ConvertAngle(float value, pinpoint_angle_unit_t from_unit, pinpoint_angle_unit_t to_unit)
{
    if (from_unit == to_unit) return value;
    
    if (from_unit == PINPOINT_ANGLE_DEGREES && to_unit == PINPOINT_ANGLE_RADIANS) {
        return value * DEG_TO_RAD;
    } else if (from_unit == PINPOINT_ANGLE_RADIANS && to_unit == PINPOINT_ANGLE_DEGREES) {
        return value * RAD_TO_DEG;
    }
    
    return value;
}

/* DMA Functions Implementation */

/**
 * @brief Start DMA read of all sensor data
 */
HAL_StatusTypeDef Pinpoint_StartDMARead(pinpoint_handle_t *handle)
{
    if (handle == NULL || handle->hi2c == NULL) {
        return HAL_ERROR;
    }
    
    // Check if DMA is already in progress
    if (handle->dma_state == PINPOINT_DMA_READING) {
        return HAL_BUSY;
    }
    
    // Reset DMA state
    handle->dma_state = PINPOINT_DMA_READING;
    handle->dma_read_complete = false;
    
    // Clear DMA buffer
    memset(handle->dma_buffer, 0, sizeof(handle->dma_buffer));
    
    // Start with register address for encoder values - 從編碼器暫存器開始讀取
    uint8_t reg_addr = (uint8_t)PINPOINT_REG_X_ENCODER_VALUE;  // 寄存器 6
    
    // Use I2C Memory Read DMA for proper I2C protocol
    // 讀取從編碼器值開始的連續數據 (寄存器 6-13)
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(handle->hi2c, 
                                                     handle->device_address << 1, 
                                                     reg_addr,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     handle->dma_buffer, 
                                                     32);  // 讀取 32 字節 (8個寄存器 x 4字節)
    if (status != HAL_OK) {
        handle->dma_state = PINPOINT_DMA_ERROR;
        return status;
    }
    
    return HAL_OK;
}

/**
 * @brief Check if DMA read is complete
 */
bool Pinpoint_IsDMAComplete(pinpoint_handle_t *handle)
{
    if (handle == NULL) {
        return false;
    }
    
    return handle->dma_read_complete;
}

/**
 * @brief Process DMA read data and update handle
 */
HAL_StatusTypeDef Pinpoint_ProcessDMAData(pinpoint_handle_t *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }
    
    if (handle->dma_state != PINPOINT_DMA_COMPLETE) {
        return HAL_BUSY;
    }
    
    // Parse the DMA buffer data - 從編碼器值開始 (寄存器 6)
    uint8_t *data = handle->dma_buffer;
    
    // Extract encoder values (4 bytes each, starting from register 6) - 使用與同步讀取相同的字節序
    // 注意：可能需要交換 X/Y 軸以匹配實際的機械配置
    uint32_t x_enc_raw = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    uint32_t y_enc_raw = (uint32_t)(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
    
    // 檢查 X 軸編碼器數據是否異常（可能導致卡死的原因）
    int32_t new_x_enc = *(int32_t*)&y_enc_raw;   // 實際的 X 軸對應 Pinpoint 的 Y 寄存器
    int32_t new_y_enc = *(int32_t*)&x_enc_raw;   // 實際的 Y 軸對應 Pinpoint 的 X 寄存器
    
    // 檢查 X 軸編碼器變化是否過大（可能表示硬體問題）
    static int32_t last_x_enc = 0;
    static bool x_enc_initialized = false;
    
    if (!x_enc_initialized) {
        last_x_enc = new_x_enc;
        x_enc_initialized = true;
    }
    
    int32_t x_enc_delta = new_x_enc - last_x_enc;
    
    // 如果 X 軸編碼器變化過大，可能有問題，暫時跳過此次更新
    if (abs(x_enc_delta) > 10000) {
        // X 軸編碼器異常，保持舊值
        new_x_enc = last_x_enc;
    } else {
        last_x_enc = new_x_enc;
    }
    
    // 軸向修正：X 軸反向，Y 軸直接對應（與同步模式一致）
    handle->x_encoder_value = -new_x_enc;  // X 軸 = -X 寄存器（反向）
    handle->y_encoder_value = new_y_enc;   // Y 軸 = Y 寄存器（直接對應）
    
    // Extract position data (floats, 4 bytes each, starting from register 8) - 使用正確的字節序
    // 同樣交換 X/Y 軸映射以匹配實際機械配置
    uint32_t x_pos_raw = (uint32_t)(data[8] | (data[9] << 8) | (data[10] << 16) | (data[11] << 24));
    uint32_t y_pos_raw = (uint32_t)(data[12] | (data[13] << 8) | (data[14] << 16) | (data[15] << 24));
    uint32_t h_pos_raw = (uint32_t)(data[16] | (data[17] << 8) | (data[18] << 16) | (data[19] << 24));
    handle->current_pose.x = -(*(float*)&x_pos_raw);     // X 位置 = -X 寄存器（反向）
    handle->current_pose.y = *(float*)&y_pos_raw;        // Y 位置 = Y 寄存器（直接對應）
    handle->current_pose.heading = *(float*)&h_pos_raw;  // H_ORIENTATION (reg 10)
    
    // Extract velocity data (floats, 4 bytes each, starting from register 11) - 使用正確的字節序
    // 同樣交換 X/Y 軸映射以匹配實際機械配置
    uint32_t x_vel_raw = (uint32_t)(data[20] | (data[21] << 8) | (data[22] << 16) | (data[23] << 24));
    uint32_t y_vel_raw = (uint32_t)(data[24] | (data[25] << 8) | (data[26] << 16) | (data[27] << 24));
    uint32_t h_vel_raw = (uint32_t)(data[28] | (data[29] << 8) | (data[30] << 16) | (data[31] << 24));
    handle->current_velocity.x_velocity = -(*(float*)&x_vel_raw);  // X 速度 = -X 寄存器（反向）
    handle->current_velocity.y_velocity = *(float*)&y_vel_raw;   // Y 速度 = Y 寄存器（直接對應）
    handle->current_velocity.h_velocity = *(float*)&h_vel_raw;  // H_VELOCITY (reg 13)
    
    // Read device status separately since it may not be contiguous
    HAL_StatusTypeDef status = pinpoint_read_byte(handle, PINPOINT_REG_DEVICE_STATUS, &handle->device_status);
    
    // Reset DMA state for next read
    handle->dma_state = PINPOINT_DMA_IDLE;
    handle->dma_read_complete = false;
    
    return status;
}

/**
 * @brief DMA transfer complete callback
 */
void Pinpoint_DMACallback(pinpoint_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    handle->dma_state = PINPOINT_DMA_COMPLETE;
    handle->dma_read_complete = true;
}
