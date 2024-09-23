################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc_ads130b04.c \
../Core/Src/dac_dac63204.c \
../Core/Src/fram_fm24cl64.c \
../Core/Src/led_task.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/user_mmi.c \
../Core/Src/user_uart.c \
../Core/Src/util_debug.c 

OBJS += \
./Core/Src/adc_ads130b04.o \
./Core/Src/dac_dac63204.o \
./Core/Src/fram_fm24cl64.o \
./Core/Src/led_task.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/user_mmi.o \
./Core/Src/user_uart.o \
./Core/Src/util_debug.o 

C_DEPS += \
./Core/Src/adc_ads130b04.d \
./Core/Src/dac_dac63204.d \
./Core/Src/fram_fm24cl64.d \
./Core/Src/led_task.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/user_mmi.d \
./Core/Src/user_uart.d \
./Core/Src/util_debug.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc_ads130b04.cyclo ./Core/Src/adc_ads130b04.d ./Core/Src/adc_ads130b04.o ./Core/Src/adc_ads130b04.su ./Core/Src/dac_dac63204.cyclo ./Core/Src/dac_dac63204.d ./Core/Src/dac_dac63204.o ./Core/Src/dac_dac63204.su ./Core/Src/fram_fm24cl64.cyclo ./Core/Src/fram_fm24cl64.d ./Core/Src/fram_fm24cl64.o ./Core/Src/fram_fm24cl64.su ./Core/Src/led_task.cyclo ./Core/Src/led_task.d ./Core/Src/led_task.o ./Core/Src/led_task.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/user_mmi.cyclo ./Core/Src/user_mmi.d ./Core/Src/user_mmi.o ./Core/Src/user_mmi.su ./Core/Src/user_uart.cyclo ./Core/Src/user_uart.d ./Core/Src/user_uart.o ./Core/Src/user_uart.su ./Core/Src/util_debug.cyclo ./Core/Src/util_debug.d ./Core/Src/util_debug.o ./Core/Src/util_debug.su

.PHONY: clean-Core-2f-Src

