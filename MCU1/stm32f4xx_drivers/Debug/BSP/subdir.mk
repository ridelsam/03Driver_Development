################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/ds1307.c \
../BSP/lcd.c 

OBJS += \
./BSP/ds1307.o \
./BSP/lcd.o 

C_DEPS += \
./BSP/ds1307.d \
./BSP/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/rsamonte/Embedded_Systems/Driver Development/MCU1/stm32f4xx_drivers/drivers/Inc" -I"C:/Users/rsamonte/Embedded_Systems/Driver Development/MCU1/stm32f4xx_drivers/BSP" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/ds1307.d ./BSP/ds1307.o ./BSP/lcd.d ./BSP/lcd.o

.PHONY: clean-BSP

