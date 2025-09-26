################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ROS1/ros1.cpp \
../ROS1/ros_test_minimal.cpp 

OBJS += \
./ROS1/ros1.o \
./ROS1/ros_test_minimal.o 

CPP_DEPS += \
./ROS1/ros1.d \
./ROS1/ros_test_minimal.d 


# Each subdirectory must supply rules for building sources it contributes
ROS1/%.o ROS1/%.su ROS1/%.cyclo: ../ROS1/%.cpp ROS1/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Chassis -I../Motor -I../ROS1 -I../location -I../Drivers/VL53L0X -I../Lifter -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ROS1

clean-ROS1:
	-$(RM) ./ROS1/ros1.cyclo ./ROS1/ros1.d ./ROS1/ros1.o ./ROS1/ros1.su ./ROS1/ros_test_minimal.cyclo ./ROS1/ros_test_minimal.d ./ROS1/ros_test_minimal.o ./ROS1/ros_test_minimal.su

.PHONY: clean-ROS1

