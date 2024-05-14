################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.c \
../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.c 

OBJS += \
./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.o \
./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.o 

C_DEPS += \
./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.d \
./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L4CD_ULD_Driver/%.o Drivers/VL53L4CD_ULD_Driver/%.su Drivers/VL53L4CD_ULD_Driver/%.cyclo: ../Drivers/VL53L4CD_ULD_Driver/%.c Drivers/VL53L4CD_ULD_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/jonat/STM32CubeIDE/workspace_1.13.2/RobotCar_v2.0.0/Drivers/VL53L4CD_ULD_Driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L4CD_ULD_Driver

clean-Drivers-2f-VL53L4CD_ULD_Driver:
	-$(RM) ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.cyclo ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.d ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.o ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.su ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.cyclo ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.d ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.o ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.su

.PHONY: clean-Drivers-2f-VL53L4CD_ULD_Driver

