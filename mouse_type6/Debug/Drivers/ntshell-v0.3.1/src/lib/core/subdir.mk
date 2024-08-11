################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ntshell-v0.3.1/src/lib/core/ntlibc.c \
../Drivers/ntshell-v0.3.1/src/lib/core/ntshell.c \
../Drivers/ntshell-v0.3.1/src/lib/core/text_editor.c \
../Drivers/ntshell-v0.3.1/src/lib/core/text_history.c \
../Drivers/ntshell-v0.3.1/src/lib/core/vtrecv.c \
../Drivers/ntshell-v0.3.1/src/lib/core/vtsend.c 

OBJS += \
./Drivers/ntshell-v0.3.1/src/lib/core/ntlibc.o \
./Drivers/ntshell-v0.3.1/src/lib/core/ntshell.o \
./Drivers/ntshell-v0.3.1/src/lib/core/text_editor.o \
./Drivers/ntshell-v0.3.1/src/lib/core/text_history.o \
./Drivers/ntshell-v0.3.1/src/lib/core/vtrecv.o \
./Drivers/ntshell-v0.3.1/src/lib/core/vtsend.o 

C_DEPS += \
./Drivers/ntshell-v0.3.1/src/lib/core/ntlibc.d \
./Drivers/ntshell-v0.3.1/src/lib/core/ntshell.d \
./Drivers/ntshell-v0.3.1/src/lib/core/text_editor.d \
./Drivers/ntshell-v0.3.1/src/lib/core/text_history.d \
./Drivers/ntshell-v0.3.1/src/lib/core/vtrecv.d \
./Drivers/ntshell-v0.3.1/src/lib/core/vtsend.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ntshell-v0.3.1/src/lib/core/ntlibc.o: ../Drivers/ntshell-v0.3.1/src/lib/core/ntlibc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/core/ntlibc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/ntshell-v0.3.1/src/lib/core/ntshell.o: ../Drivers/ntshell-v0.3.1/src/lib/core/ntshell.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/core/ntshell.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/ntshell-v0.3.1/src/lib/core/text_editor.o: ../Drivers/ntshell-v0.3.1/src/lib/core/text_editor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/core/text_editor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/ntshell-v0.3.1/src/lib/core/text_history.o: ../Drivers/ntshell-v0.3.1/src/lib/core/text_history.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/core/text_history.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/ntshell-v0.3.1/src/lib/core/vtrecv.o: ../Drivers/ntshell-v0.3.1/src/lib/core/vtrecv.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/core/vtrecv.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/ntshell-v0.3.1/src/lib/core/vtsend.o: ../Drivers/ntshell-v0.3.1/src/lib/core/vtsend.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ntshell-v0.3.1/src/lib/core/vtsend.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

