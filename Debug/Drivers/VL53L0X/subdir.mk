################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L0X/vl53l0x_api.c \
../Drivers/VL53L0X/vl53l0x_api_calibration.c \
../Drivers/VL53L0X/vl53l0x_api_core.c \
../Drivers/VL53L0X/vl53l0x_api_ranging.c \
../Drivers/VL53L0X/vl53l0x_api_strings.c \
../Drivers/VL53L0X/vl53l0x_platform.c \
../Drivers/VL53L0X/vl53l0x_platform_log.c 

C_DEPS += \
./Drivers/VL53L0X/vl53l0x_api.d \
./Drivers/VL53L0X/vl53l0x_api_calibration.d \
./Drivers/VL53L0X/vl53l0x_api_core.d \
./Drivers/VL53L0X/vl53l0x_api_ranging.d \
./Drivers/VL53L0X/vl53l0x_api_strings.d \
./Drivers/VL53L0X/vl53l0x_platform.d \
./Drivers/VL53L0X/vl53l0x_platform_log.d 

OBJS += \
./Drivers/VL53L0X/vl53l0x_api.o \
./Drivers/VL53L0X/vl53l0x_api_calibration.o \
./Drivers/VL53L0X/vl53l0x_api_core.o \
./Drivers/VL53L0X/vl53l0x_api_ranging.o \
./Drivers/VL53L0X/vl53l0x_api_strings.o \
./Drivers/VL53L0X/vl53l0x_platform.o \
./Drivers/VL53L0X/vl53l0x_platform_log.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X/%.o Drivers/VL53L0X/%.su Drivers/VL53L0X/%.cyclo: ../Drivers/VL53L0X/%.c Drivers/VL53L0X/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X

clean-Drivers-2f-VL53L0X:
	-$(RM) ./Drivers/VL53L0X/vl53l0x_api.cyclo ./Drivers/VL53L0X/vl53l0x_api.d ./Drivers/VL53L0X/vl53l0x_api.o ./Drivers/VL53L0X/vl53l0x_api.su ./Drivers/VL53L0X/vl53l0x_api_calibration.cyclo ./Drivers/VL53L0X/vl53l0x_api_calibration.d ./Drivers/VL53L0X/vl53l0x_api_calibration.o ./Drivers/VL53L0X/vl53l0x_api_calibration.su ./Drivers/VL53L0X/vl53l0x_api_core.cyclo ./Drivers/VL53L0X/vl53l0x_api_core.d ./Drivers/VL53L0X/vl53l0x_api_core.o ./Drivers/VL53L0X/vl53l0x_api_core.su ./Drivers/VL53L0X/vl53l0x_api_ranging.cyclo ./Drivers/VL53L0X/vl53l0x_api_ranging.d ./Drivers/VL53L0X/vl53l0x_api_ranging.o ./Drivers/VL53L0X/vl53l0x_api_ranging.su ./Drivers/VL53L0X/vl53l0x_api_strings.cyclo ./Drivers/VL53L0X/vl53l0x_api_strings.d ./Drivers/VL53L0X/vl53l0x_api_strings.o ./Drivers/VL53L0X/vl53l0x_api_strings.su ./Drivers/VL53L0X/vl53l0x_platform.cyclo ./Drivers/VL53L0X/vl53l0x_platform.d ./Drivers/VL53L0X/vl53l0x_platform.o ./Drivers/VL53L0X/vl53l0x_platform.su ./Drivers/VL53L0X/vl53l0x_platform_log.cyclo ./Drivers/VL53L0X/vl53l0x_platform_log.d ./Drivers/VL53L0X/vl53l0x_platform_log.o ./Drivers/VL53L0X/vl53l0x_platform_log.su

.PHONY: clean-Drivers-2f-VL53L0X

