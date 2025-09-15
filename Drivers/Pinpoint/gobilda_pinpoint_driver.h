/*
 * gobilda_pinpoint_driver.h
 *
 * STM32 HAL Driver for goBILDA Pinpoint OTOS (Optical Tracking Odometry Sensor)
 * Based on Product 3110-0002-0001 User Guide
 *
 * Created on: Sep 1, 2025
 * Author: Your Name
 */

#ifndef GOBILDA_PINPOINT_DRIVER_H_
#define GOBILDA_PINPOINT_DRIVER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default I2C Address */
#define PINPOINT_DEFAULT_I2C_ADDR       0x31

/* Register Map */
typedef enum {
    PINPOINT_REG_DEVICE_ID          = 1,
    PINPOINT_REG_DEVICE_VERSION     = 2,
    PINPOINT_REG_DEVICE_STATUS      = 3,
    PINPOINT_REG_DEVICE_CONTROL     = 4,
    PINPOINT_REG_LOOP_TIME          = 5,
    PINPOINT_REG_X_ENCODER_VALUE    = 6,
    PINPOINT_REG_Y_ENCODER_VALUE    = 7,
    PINPOINT_REG_X_POSITION         = 8,
    PINPOINT_REG_Y_POSITION         = 9,
    PINPOINT_REG_H_ORIENTATION      = 10,
    PINPOINT_REG_X_VELOCITY         = 11,
    PINPOINT_REG_Y_VELOCITY         = 12,
    PINPOINT_REG_H_VELOCITY         = 13,
    PINPOINT_REG_MM_PER_TICK        = 14,
    PINPOINT_REG_X_POD_OFFSET       = 15,
    PINPOINT_REG_Y_POD_OFFSET       = 16,
    PINPOINT_REG_YAW_SCALAR         = 17,
    PINPOINT_REG_BULK_READ          = 18
} pinpoint_register_t;

/* Device Status Flags */
typedef enum {
    PINPOINT_STATUS_NOT_READY               = 0,
    PINPOINT_STATUS_READY                   = (1 << 0),
    PINPOINT_STATUS_CALIBRATING             = (1 << 1),
    PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED = (1 << 2),
    PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED = (1 << 3),
    PINPOINT_STATUS_FAULT_NO_PODS_DETECTED  = (PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED | PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED),
    PINPOINT_STATUS_FAULT_IMU_RUNAWAY       = (1 << 4),
    PINPOINT_STATUS_FAULT_BAD_READ          = (1 << 5)
} pinpoint_status_t;

/* Encoder Direction */
typedef enum {
    PINPOINT_ENCODER_FORWARD = 0,
    PINPOINT_ENCODER_REVERSED = 1
} pinpoint_encoder_direction_t;

/* goBILDA Odometry Pod Types */
typedef enum {
    GOBILDA_SWINGARM_POD = 0,   // 13.26291192 ticks/mm
    GOBILDA_4_BAR_POD = 1       // 19.89436789 ticks/mm
} gobilda_pod_type_t;

/* Distance Units */
typedef enum {
    PINPOINT_UNIT_MM = 0,
    PINPOINT_UNIT_CM = 1,
    PINPOINT_UNIT_M = 2,
    PINPOINT_UNIT_INCH = 3
} pinpoint_distance_unit_t;

/* Angle Units */
typedef enum {
    PINPOINT_ANGLE_RADIANS = 0,
    PINPOINT_ANGLE_DEGREES = 1
} pinpoint_angle_unit_t;

/* Position Structure */
typedef struct {
    float x;            // X position
    float y;            // Y position
    float heading;      // Heading angle
} pinpoint_pose_t;

/* Velocity Structure */
typedef struct {
    float x_velocity;   // X velocity
    float y_velocity;   // Y velocity
    float h_velocity;   // Heading velocity
} pinpoint_velocity_t;

/* DMA Read State */
typedef enum {
    PINPOINT_DMA_IDLE = 0,
    PINPOINT_DMA_READING,
    PINPOINT_DMA_COMPLETE,
    PINPOINT_DMA_ERROR
} pinpoint_dma_state_t;

/* Pinpoint Driver Handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;                    // I2C handle
    uint8_t device_address;                     // I2C device address
    
    // Configuration
    float x_pod_offset;                         // X pod offset in mm
    float y_pod_offset;                         // Y pod offset in mm
    float mm_per_tick;                          // Encoder resolution
    float yaw_scalar;                           // Yaw correction scalar
    
    // Current readings
    pinpoint_pose_t current_pose;               // Current position
    pinpoint_velocity_t current_velocity;       // Current velocity
    int32_t x_encoder_value;                    // Raw X encoder value
    int32_t y_encoder_value;                    // Raw Y encoder value
    uint32_t loop_time;                         // Device loop time in microseconds
    uint8_t device_status;                      // Device status flags
    
    // DMA support
    uint8_t dma_buffer[32];                     // DMA read buffer
    pinpoint_dma_state_t dma_state;             // DMA operation state
    volatile bool dma_read_complete;            // DMA completion flag
    
    // Settings
    pinpoint_distance_unit_t distance_unit;    // Distance unit preference
    pinpoint_angle_unit_t angle_unit;           // Angle unit preference
    
} pinpoint_handle_t;

/* Function Prototypes */

/**
 * @brief Initialize the Pinpoint driver
 * @param handle Pointer to pinpoint handle structure
 * @param hi2c Pointer to I2C handle
 * @param device_addr I2C device address (7-bit)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Init(pinpoint_handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t device_addr);

/**
 * @brief Reset the Pinpoint device
 * @param handle Pointer to pinpoint handle structure
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Reset(pinpoint_handle_t *handle);

/**
 * @brief Set encoder pod offsets
 * @param handle Pointer to pinpoint handle structure
 * @param x_offset X pod offset in mm (left positive, right negative)
 * @param y_offset Y pod offset in mm (forward positive, backward negative)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_SetOffsets(pinpoint_handle_t *handle, float x_offset, float y_offset);

/**
 * @brief Set encoder resolution using goBILDA pod type
 * @param handle Pointer to pinpoint handle structure
 * @param pod_type goBILDA pod type
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_SetEncoderResolution_goBILDA(pinpoint_handle_t *handle, gobilda_pod_type_t pod_type);

/**
 * @brief Set encoder resolution manually
 * @param handle Pointer to pinpoint handle structure
 * @param mm_per_tick Millimeters per encoder tick
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_SetEncoderResolution(pinpoint_handle_t *handle, float mm_per_tick);

/**
 * @brief Set encoder directions
 * @param handle Pointer to pinpoint handle structure
 * @param x_direction X encoder direction
 * @param y_direction Y encoder direction
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_SetEncoderDirections(pinpoint_handle_t *handle, 
                                                pinpoint_encoder_direction_t x_direction,
                                                pinpoint_encoder_direction_t y_direction);

/**
 * @brief Set yaw scalar for heading correction
 * @param handle Pointer to pinpoint handle structure
 * @param yaw_scalar Yaw correction scalar (typically 1.0)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_SetYawScalar(pinpoint_handle_t *handle, float yaw_scalar);

/**
 * @brief Update all sensor readings
 * @param handle Pointer to pinpoint handle structure
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Update(pinpoint_handle_t *handle);

/**
 * @brief Recalibrate the IMU
 * @param handle Pointer to pinpoint handle structure
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_RecalibrateIMU(pinpoint_handle_t *handle);

/**
 * @brief Reset position to (0, 0, 0)
 * @param handle Pointer to pinpoint handle structure
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_ResetPosition(pinpoint_handle_t *handle);

/**
 * @brief Set position manually
 * @param handle Pointer to pinpoint handle structure
 * @param x X position
 * @param y Y position  
 * @param heading Heading angle
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_SetPosition(pinpoint_handle_t *handle, float x, float y, float heading);

/**
 * @brief Get current position
 * @param handle Pointer to pinpoint handle structure
 * @param pose Pointer to pose structure to fill
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_GetPosition(pinpoint_handle_t *handle, pinpoint_pose_t *pose);

/**
 * @brief Get current velocity
 * @param handle Pointer to pinpoint handle structure
 * @param velocity Pointer to velocity structure to fill
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_GetVelocity(pinpoint_handle_t *handle, pinpoint_velocity_t *velocity);

/**
 * @brief Get device status
 * @param handle Pointer to pinpoint handle structure
 * @param status Pointer to status variable
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_GetDeviceStatus(pinpoint_handle_t *handle, uint8_t *status);

/**
 * @brief Check if device is ready
 * @param handle Pointer to pinpoint handle structure
 * @return true if ready, false otherwise
 */
bool Pinpoint_IsReady(pinpoint_handle_t *handle);

/**
 * @brief Check if device has any faults
 * @param handle Pointer to pinpoint handle structure
 * @return true if faults detected, false otherwise
 */
bool Pinpoint_HasFaults(pinpoint_handle_t *handle);

/**
 * @brief Get device ID
 * @param handle Pointer to pinpoint handle structure
 * @param device_id Pointer to device ID variable
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_GetDeviceID(pinpoint_handle_t *handle, uint8_t *device_id);

/**
 * @brief Get device version
 * @param handle Pointer to pinpoint handle structure
 * @param version Pointer to version variable
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_GetDeviceVersion(pinpoint_handle_t *handle, uint8_t *version);

/**
 * @brief Convert distance units
 * @param value Input value
 * @param from_unit Source unit
 * @param to_unit Target unit
 * @return Converted value
 */
float Pinpoint_ConvertDistance(float value, pinpoint_distance_unit_t from_unit, pinpoint_distance_unit_t to_unit);

/**
 * @brief Convert angle units
 * @param value Input value
 * @param from_unit Source unit
 * @param to_unit Target unit
 * @return Converted value
 */
float Pinpoint_ConvertAngle(float value, pinpoint_angle_unit_t from_unit, pinpoint_angle_unit_t to_unit);

/* DMA Functions */

/**
 * @brief Start DMA read of all sensor data
 * @param handle Pointer to pinpoint handle structure
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_StartDMARead(pinpoint_handle_t *handle);

/**
 * @brief Check if DMA read is complete
 * @param handle Pointer to pinpoint handle structure
 * @return true if complete, false otherwise
 */
bool Pinpoint_IsDMAComplete(pinpoint_handle_t *handle);

/**
 * @brief Process DMA read data and update handle
 * @param handle Pointer to pinpoint handle structure
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_ProcessDMAData(pinpoint_handle_t *handle);

/**
 * @brief DMA transfer complete callback (called from interrupt)
 * @param handle Pointer to pinpoint handle structure
 */
void Pinpoint_DMACallback(pinpoint_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* GOBILDA_PINPOINT_DRIVER_H_ */
