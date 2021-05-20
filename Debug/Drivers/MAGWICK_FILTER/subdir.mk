################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MAGWICK_FILTER/madgwick.c 

C_DEPS += \
./Drivers/MAGWICK_FILTER/madgwick.d 

OBJS += \
./Drivers/MAGWICK_FILTER/madgwick.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MAGWICK_FILTER/madgwick.o: ../Drivers/MAGWICK_FILTER/madgwick.c Drivers/MAGWICK_FILTER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/MAGWICK_FILTER -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/MAGWICK_FILTER/madgwick.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

