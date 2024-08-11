################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/CPP/Component/codegen/rtwhalf.cpp 

OBJS += \
./Core/CPP/Component/codegen/rtwhalf.o 

CPP_DEPS += \
./Core/CPP/Component/codegen/rtwhalf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CPP/Component/codegen/rtwhalf.o: ../Core/CPP/Component/codegen/rtwhalf.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Core/CPP/Component/codegen/rtwhalf.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

