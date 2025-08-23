################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Chassis/chassis.cpp 

OBJS += \
./Chassis/chassis.o 

CPP_DEPS += \
./Chassis/chassis.d 


# Each subdirectory must supply rules for building sources it contributes
Chassis/%.o Chassis/%.su Chassis/%.cyclo: ../Chassis/%.cpp Chassis/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Chassis -I../Motor -I../ROS1 -I../location -I../Drivers/VL53L0X -I../Lifter -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Chassis

clean-Chassis:
	-$(RM) ./Chassis/chassis.cyclo ./Chassis/chassis.d ./Chassis/chassis.o ./Chassis/chassis.su

.PHONY: clean-Chassis

