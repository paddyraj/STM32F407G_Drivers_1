################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Startup/startup_stm32f407vgtx.s 

OBJS += \
./Startup/startup_stm32f407vgtx.o 

S_DEPS += \
./Startup/startup_stm32f407vgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Startup/%.o: ../Startup/%.s Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g -DDEBUG -c -I"C:/Users/Raj paddy/Desktop/Embedded Systems/STM32/MCU-1/STM32F407V_drivers/Src" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Startup

clean-Startup:
	-$(RM) ./Startup/startup_stm32f407vgtx.d ./Startup/startup_stm32f407vgtx.o

.PHONY: clean-Startup

