################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CPP/src/mainpp.cpp 

OBJS += \
./CPP/src/mainpp.o 

CPP_DEPS += \
./CPP/src/mainpp.d 


# Each subdirectory must supply rules for building sources it contributes
CPP/src/mainpp.o: ../CPP/src/mainpp.cpp CPP/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/MAGWICK_FILTER -I../Drivers/CMSIS/Include -I../CPP/inc -I../Drivers/COMPONENTS/test -I../Drivers/COMPONENTS/MPU9250 -I../Drivers/TJ_MPU6050 -I../Drivers/COMPONENTS/ROS -I../Drivers/rosserial -I../Drivers/rosserial/include -I../Drivers/COMPONENTS/MADGWICK -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"CPP/src/mainpp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

