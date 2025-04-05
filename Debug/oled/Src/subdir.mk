################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../oled/Src/oled.c 

OBJS += \
./oled/Src/oled.o 

C_DEPS += \
./oled/Src/oled.d 


# Each subdirectory must supply rules for building sources it contributes
oled/Src/%.o oled/Src/%.su oled/Src/%.cyclo: ../oled/Src/%.c oled/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../oled/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-oled-2f-Src

clean-oled-2f-Src:
	-$(RM) ./oled/Src/oled.cyclo ./oled/Src/oled.d ./oled/Src/oled.o ./oled/Src/oled.su

.PHONY: clean-oled-2f-Src

