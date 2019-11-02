################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Src/main.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Src/stm32f3xx_hal_msp.c \
C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Src/stm32f3xx_it.c 

OBJS += \
./Application/User/main.o \
./Application/User/stm32f3xx_hal_msp.o \
./Application/User/stm32f3xx_it.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/stm32f3xx_hal_msp.d \
./Application/User/stm32f3xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/main.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f3xx_hal_msp.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Src/stm32f3xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32f3xx_hal_msp.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f3xx_it.o: C:/Users/kacpe/OneDrive/Dokumenty/Twoja\ stara/ZajebistyLF/Src/stm32f3xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xC -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/kacpe/OneDrive/Dokumenty/Twoja stara/ZajebistyLF/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Application/User/stm32f3xx_it.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


