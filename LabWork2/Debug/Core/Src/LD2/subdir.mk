################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LD2/main.c 

OBJS += \
./Core/Src/LD2/main.o 

C_DEPS += \
./Core/Src/LD2/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LD2/%.o Core/Src/LD2/%.su Core/Src/LD2/%.cyclo: ../Core/Src/LD2/%.c Core/Src/LD2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LD2

clean-Core-2f-Src-2f-LD2:
	-$(RM) ./Core/Src/LD2/main.cyclo ./Core/Src/LD2/main.d ./Core/Src/LD2/main.o ./Core/Src/LD2/main.su

.PHONY: clean-Core-2f-Src-2f-LD2

