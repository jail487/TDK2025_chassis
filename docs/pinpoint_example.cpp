/*
 * pinpoint_example.c
 *
 * Example usage of goBILDA Pinpoint OTOS driver for STM32
 * This file demonstrates how to integrate the Pinpoint sensor with your chassis system
 *
 * Created on: Sep 1, 2025
 * Author: Your Name
 */

#include "gobilda_pinpoint_driver.h"
#include "main.h"
#include <stdio.h>

/* External I2C handle - adjust according to your configuration */
extern I2C_HandleTypeDef hi2c1;

/* Pinpoint sensor handle */
static pinpoint_handle_t pinpoint;

/* Example configuration values from user guide */
#define PINPOINT_X_OFFSET_MM    -84.0f      // X pod offset (tuned for 3110-0002-0001)
#define PINPOINT_Y_OFFSET_MM    -168.0f     // Y pod offset (tuned for 3110-0002-0001)

/**
 * @brief Initialize Pinpoint sensor for chassis odometry
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Chassis_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t device_id, device_version;
    
    // Initialize the Pinpoint driver
    status = Pinpoint_Init(&pinpoint, &hi2c1, PINPOINT_DEFAULT_I2C_ADDR);
    if (status != HAL_OK) {
        printf("Pinpoint Init Failed: %d\n", status);
        return status;
    }
    
    // Get device information
    status = Pinpoint_GetDeviceID(&pinpoint, &device_id);
    if (status == HAL_OK) {
        printf("Pinpoint Device ID: 0x%02X\n", device_id);
    }
    
    status = Pinpoint_GetDeviceVersion(&pinpoint, &device_version);
    if (status == HAL_OK) {
        printf("Pinpoint Version: %d\n", device_version);
    }
    
    // Set encoder pod offsets (adjust based on your robot geometry)
    status = Pinpoint_SetOffsets(&pinpoint, PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM);
    if (status != HAL_OK) {
        printf("Failed to set offsets: %d\n", status);
        return status;
    }
    
    // Set encoder resolution for goBILDA 4-bar pod (adjust for your pod type)
    status = Pinpoint_SetEncoderResolution_goBILDA(&pinpoint, GOBILDA_4_BAR_POD);
    if (status != HAL_OK) {
        printf("Failed to set encoder resolution: %d\n", status);
        return status;
    }
    
    // Set encoder directions (adjust based on your robot's encoder mounting)
    status = Pinpoint_SetEncoderDirections(&pinpoint, PINPOINT_ENCODER_FORWARD, PINPOINT_ENCODER_FORWARD);
    if (status != HAL_OK) {
        printf("Failed to set encoder directions: %d\n", status);
        return status;
    }
    
    // Set yaw scalar (typically 1.0, adjust if needed for heading accuracy)
    status = Pinpoint_SetYawScalar(&pinpoint, 1.0f);
    if (status != HAL_OK) {
        printf("Failed to set yaw scalar: %d\n", status);
        return status;
    }
    
    // Wait for device to be ready
    int timeout = 100; // 10 seconds timeout
    while (!Pinpoint_IsReady(&pinpoint) && timeout > 0) {
        HAL_Delay(100);
        Pinpoint_Update(&pinpoint);
        timeout--;
    }
    
    if (!Pinpoint_IsReady(&pinpoint)) {
        printf("Pinpoint not ready after timeout\n");
        return HAL_TIMEOUT;
    }
    
    // Reset position to origin
    status = Pinpoint_ResetPosition(&pinpoint);
    if (status != HAL_OK) {
        printf("Failed to reset position: %d\n", status);
        return status;
    }
    
    printf("Pinpoint initialization completed successfully\n");
    return HAL_OK;
}

/**
 * @brief Update chassis position using Pinpoint sensor
 * Call this function in your main loop or timer interrupt
 */
void Pinpoint_Chassis_Update(void)
{
    HAL_StatusTypeDef status;
    pinpoint_pose_t pose;
    pinpoint_velocity_t velocity;
    uint8_t device_status;
    
    // Update sensor readings
    status = Pinpoint_Update(&pinpoint);
    if (status != HAL_OK) {
        printf("Pinpoint update failed: %d\n", status);
        return;
    }
    
    // Check for faults
    if (Pinpoint_HasFaults(&pinpoint)) {
        Pinpoint_GetDeviceStatus(&pinpoint, &device_status);
        printf("Pinpoint fault detected: 0x%02X\n", device_status);
        
        if (device_status & PINPOINT_STATUS_FAULT_X_POD_NOT_DETECTED) {
            printf("  - X pod not detected\n");
        }
        if (device_status & PINPOINT_STATUS_FAULT_Y_POD_NOT_DETECTED) {
            printf("  - Y pod not detected\n");
        }
        if (device_status & PINPOINT_STATUS_FAULT_IMU_RUNAWAY) {
            printf("  - IMU runaway\n");
        }
        if (device_status & PINPOINT_STATUS_FAULT_BAD_READ) {
            printf("  - Bad read\n");
        }
        return;
    }
    
    // Get current position and velocity
    Pinpoint_GetPosition(&pinpoint, &pose);
    Pinpoint_GetVelocity(&pinpoint, &velocity);
    
    // Update your global position variables (adjust variable names as needed)
    // extern float map_x, map_y, theta; // Your global position variables
    // map_x = pose.x;
    // map_y = pose.y;
    // theta = pose.heading;
    
    // Optional: Print debug information (remove in production)
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print > 1000) { // Print every 1 second
        printf("Position: X=%.2f, Y=%.2f, H=%.2f deg\n", 
               pose.x, pose.y, pose.heading * 57.3f);
        printf("Velocity: Vx=%.2f, Vy=%.2f, Vh=%.2f deg/s\n", 
               velocity.x_velocity, velocity.y_velocity, velocity.h_velocity * 57.3f);
        last_print = HAL_GetTick();
    }
}

/**
 * @brief Get current chassis position from Pinpoint
 * @param x Pointer to X position (mm)
 * @param y Pointer to Y position (mm)
 * @param heading Pointer to heading (radians)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Chassis_GetPosition(float *x, float *y, float *heading)
{
    pinpoint_pose_t pose;
    HAL_StatusTypeDef status = Pinpoint_GetPosition(&pinpoint, &pose);
    
    if (status == HAL_OK) {
        *x = pose.x;
        *y = pose.y;
        *heading = pose.heading;
    }
    
    return status;
}

/**
 * @brief Set chassis position manually (for field positioning)
 * @param x X position (mm)
 * @param y Y position (mm)
 * @param heading Heading (radians)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Chassis_SetPosition(float x, float y, float heading)
{
    return Pinpoint_SetPosition(&pinpoint, x, y, heading);
}

/**
 * @brief Reset chassis position to origin
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Chassis_ResetPosition(void)
{
    return Pinpoint_ResetPosition(&pinpoint);
}

/**
 * @brief Check if Pinpoint sensor is ready and working
 * @return true if ready, false otherwise
 */
bool Pinpoint_Chassis_IsReady(void)
{
    return Pinpoint_IsReady(&pinpoint) && !Pinpoint_HasFaults(&pinpoint);
}

/**
 * @brief Recalibrate Pinpoint IMU
 * Call this when the robot is stationary to improve heading accuracy
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Pinpoint_Chassis_Recalibrate(void)
{
    printf("Recalibrating Pinpoint IMU...\n");
    HAL_StatusTypeDef status = Pinpoint_RecalibrateIMU(&pinpoint);
    
    if (status == HAL_OK) {
        // Wait for calibration to complete
        HAL_Delay(2000);
        printf("Pinpoint IMU recalibration completed\n");
    } else {
        printf("Pinpoint IMU recalibration failed: %d\n", status);
    }
    
    return status;
}

/**
 * @brief Integration example with your existing ROS system
 * This shows how to publish Pinpoint data to ROS
 */
void Pinpoint_ROS_Integration_Example(void)
{
    pinpoint_pose_t pose;
    
    // Get current position
    if (Pinpoint_GetPosition(&pinpoint, &pose) == HAL_OK) {
        // Update your ROS message data structures
        // Example (adjust to your ROS message structure):
        /*
        extern geometry_msgs::Pose chassis_current_pose;
        chassis_current_pose.position.x = pose.x / 1000.0f;  // Convert mm to m
        chassis_current_pose.position.y = pose.y / 1000.0f;  // Convert mm to m
        
        // Convert heading to quaternion (simplified - only Z rotation)
        chassis_current_pose.orientation.z = sin(pose.heading / 2.0f);
        chassis_current_pose.orientation.w = cos(pose.heading / 2.0f);
        */
    }
}

/* 
 * Usage in your main.c:
 * 
 * 1. In main() function, after I2C initialization:
 *    if (Pinpoint_Chassis_Init() != HAL_OK) {
 *        Error_Handler();
 *    }
 * 
 * 2. In your main loop or timer interrupt (e.g., 50Hz):
 *    Pinpoint_Chassis_Update();
 * 
 * 3. When you need position data:
 *    float x, y, heading;
 *    if (Pinpoint_Chassis_GetPosition(&x, &y, &heading) == HAL_OK) {
 *        // Use position data
 *    }
 */
