################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Math/Filters.c 

OBJS += \
./Core/Math/Filters.o 

C_DEPS += \
./Core/Math/Filters.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Math/%.o Core/Math/%.su Core/Math/%.cyclo: ../Core/Math/%.c Core/Math/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Core/Sensors -I../Core/Scaler -I../Core/PeripheralPorts -I../Core/Math -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Math

clean-Core-2f-Math:
	-$(RM) ./Core/Math/Filters.cyclo ./Core/Math/Filters.d ./Core/Math/Filters.o ./Core/Math/Filters.su

.PHONY: clean-Core-2f-Math

