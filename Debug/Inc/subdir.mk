################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/STM32F407_gpio_driver.c 

OBJS += \
./Inc/STM32F407_gpio_driver.o 

C_DEPS += \
./Inc/STM32F407_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/%.o Inc/%.su Inc/%.cyclo: ../Inc/%.c Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Raj paddy/Desktop/Embedded Systems/STM32/MCU-1/STM32F407V_drivers/Drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Inc

clean-Inc:
	-$(RM) ./Inc/STM32F407_gpio_driver.cyclo ./Inc/STM32F407_gpio_driver.d ./Inc/STM32F407_gpio_driver.o ./Inc/STM32F407_gpio_driver.su

.PHONY: clean-Inc

