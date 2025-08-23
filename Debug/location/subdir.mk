################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../location/VL53.cpp \
../location/location.cpp \
../location/pathsensor.cpp \
../location/script.cpp 

OBJS += \
./location/VL53.o \
./location/location.o \
./location/pathsensor.o \
./location/script.o 

CPP_DEPS += \
./location/VL53.d \
./location/location.d \
./location/pathsensor.d \
./location/script.d 


# Each subdirectory must supply rules for building sources it contributes
location/%.o location/%.su location/%.cyclo: ../location/%.cpp location/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Chassis -I../Motor -I../ROS1 -I../location -I../Drivers/VL53L0X -I../Lifter -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-location

clean-location:
	-$(RM) ./location/VL53.cyclo ./location/VL53.d ./location/VL53.o ./location/VL53.su ./location/location.cyclo ./location/location.d ./location/location.o ./location/location.su ./location/pathsensor.cyclo ./location/pathsensor.d ./location/pathsensor.o ./location/pathsensor.su ./location/script.cyclo ./location/script.d ./location/script.o ./location/script.su

.PHONY: clean-location

