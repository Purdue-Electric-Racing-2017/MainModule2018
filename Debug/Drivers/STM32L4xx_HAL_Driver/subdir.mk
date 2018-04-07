################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_can.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_cortex.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_dma.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ex.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ramfunc.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_gpio.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c_ex.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_pwr.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_pwr_ex.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_rcc.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_rcc_ex.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_tim.c \
../Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_tim_ex.c 

OBJS += \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_can.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_cortex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_dma.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ramfunc.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_gpio.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c_ex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_pwr.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_pwr_ex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_rcc.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_rcc_ex.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_tim.o \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_tim_ex.o 

C_DEPS += \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_can.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_cortex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_dma.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_flash_ramfunc.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_gpio.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_i2c_ex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_pwr.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_pwr_ex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_rcc.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_rcc_ex.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_tim.d \
./Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal_tim_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32L4xx_HAL_Driver/stm32l4xx_hal.o: C:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L432xx -I"C:/Users/ben/workspace/MainModule2018/Inc" -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/CMSIS/Include -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32L4xx_HAL_Driver/%.o: ../Drivers/STM32L4xx_HAL_Driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L432xx -I"C:/Users/ben/workspace/MainModule2018/Inc" -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/STM32L4xx_HAL_Driver/Inc -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Drivers/CMSIS/Include -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/ben/STM32Cube/Repository/STM32Cube_FW_L4_V1.8.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


