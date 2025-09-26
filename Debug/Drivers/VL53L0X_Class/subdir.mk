################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/VL53L0X_Class/VL53.cpp \
../Drivers/VL53L0X_Class/VL53L0X_Class.cpp 

OBJS += \
./Drivers/VL53L0X_Class/VL53.o \
./Drivers/VL53L0X_Class/VL53L0X_Class.o 

CPP_DEPS += \
./Drivers/VL53L0X_Class/VL53.d \
./Drivers/VL53L0X_Class/VL53L0X_Class.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X_Class/%.o Drivers/VL53L0X_Class/%.su Drivers/VL53L0X_Class/%.cyclo: ../Drivers/VL53L0X_Class/%.cpp Drivers/VL53L0X_Class/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Chassis -I../Motor -I../ROS1 -I../location -I../Drivers/VL53L0X -I../Lifter -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X_Class

clean-Drivers-2f-VL53L0X_Class:
	-$(RM) ./Drivers/VL53L0X_Class/VL53.cyclo ./Drivers/VL53L0X_Class/VL53.d ./Drivers/VL53L0X_Class/VL53.o ./Drivers/VL53L0X_Class/VL53.su ./Drivers/VL53L0X_Class/VL53L0X_Class.cyclo ./Drivers/VL53L0X_Class/VL53L0X_Class.d ./Drivers/VL53L0X_Class/VL53L0X_Class.o ./Drivers/VL53L0X_Class/VL53L0X_Class.su

.PHONY: clean-Drivers-2f-VL53L0X_Class

