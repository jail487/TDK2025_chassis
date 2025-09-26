################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Pinpoint/gobilda_pinpoint_driver.c 

C_DEPS += \
./Drivers/Pinpoint/gobilda_pinpoint_driver.d 

OBJS += \
./Drivers/Pinpoint/gobilda_pinpoint_driver.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Pinpoint/%.o Drivers/Pinpoint/%.su Drivers/Pinpoint/%.cyclo: ../Drivers/Pinpoint/%.c Drivers/Pinpoint/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Pinpoint

clean-Drivers-2f-Pinpoint:
	-$(RM) ./Drivers/Pinpoint/gobilda_pinpoint_driver.cyclo ./Drivers/Pinpoint/gobilda_pinpoint_driver.d ./Drivers/Pinpoint/gobilda_pinpoint_driver.o ./Drivers/Pinpoint/gobilda_pinpoint_driver.su

.PHONY: clean-Drivers-2f-Pinpoint

