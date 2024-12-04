################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/STM32F407_gpio_driver.c 

OBJS += \
./Drivers/src/STM32F407_gpio_driver.o 

C_DEPS += \
./Drivers/src/STM32F407_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o Drivers/src/%.su Drivers/src/%.cyclo: ../Drivers/src/%.c Drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/Raj paddy/Desktop/Embedded Systems/STM32/MCU-1/STM32F407V_drivers/Drivers/inc" -I"C:/Users/Raj paddy/Desktop/Embedded Systems/STM32/MCU-1/STM32F407V_drivers/Src" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-src

clean-Drivers-2f-src:
	-$(RM) ./Drivers/src/STM32F407_gpio_driver.cyclo ./Drivers/src/STM32F407_gpio_driver.d ./Drivers/src/STM32F407_gpio_driver.o ./Drivers/src/STM32F407_gpio_driver.su

.PHONY: clean-Drivers-2f-src

