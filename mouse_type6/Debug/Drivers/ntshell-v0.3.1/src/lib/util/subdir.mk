################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ntshell-v0.3.1/src/lib/util/ntopt.c \
../Drivers/ntshell-v0.3.1/src/lib/util/ntstdio.c 

OBJS += \
./Drivers/ntshell-v0.3.1/src/lib/util/ntopt.o \
./Drivers/ntshell-v0.3.1/src/lib/util/ntstdio.o 

C_DEPS += \
./Drivers/ntshell-v0.3.1/src/lib/util/ntopt.d \
./Drivers/ntshell-v0.3.1/src/lib/util/ntstdio.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ntshell-v0.3.1/src/lib/util/ntopt.o: ../Drivers/ntshell-v0.3.1/src/lib/util/ntopt.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/util/ntopt.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/ntshell-v0.3.1/src/lib/util/ntstdio.o: ../Drivers/ntshell-v0.3.1/src/lib/util/ntstdio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/util/ntstdio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

