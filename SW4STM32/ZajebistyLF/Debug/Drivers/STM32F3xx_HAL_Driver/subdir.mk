################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c 

OBJS += \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_cortex.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_dma.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash_ex.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_gpio.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c_ex.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr_ex.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc_ex.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim.o \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim_ex.o 

C_DEPS += \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_cortex.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_dma.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash_ex.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_gpio.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c_ex.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr_ex.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc_ex.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim.d \
./Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_cortex.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_cortex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_dma.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_dma.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash_ex.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_flash_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_gpio.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_gpio.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c_ex.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_i2c_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr_ex.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_pwr_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc_ex.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_rcc_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim_ex.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F3xx_HAL_Driver/stm32f3xx_hal_tim_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


