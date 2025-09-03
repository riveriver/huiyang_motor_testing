################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/BSP/MODBUS-LIB/Encoder485.cpp \
../Drivers/BSP/MODBUS-LIB/EncoderMBTCP.cpp \
../Drivers/BSP/MODBUS-LIB/ModbusAbsoluteEncoder.cpp \
../Drivers/BSP/MODBUS-LIB/Sensor.cpp 

C_SRCS += \
../Drivers/BSP/MODBUS-LIB/Modbus.c \
../Drivers/BSP/MODBUS-LIB/UARTCallback.c \
../Drivers/BSP/MODBUS-LIB/retarget.c 

C_DEPS += \
./Drivers/BSP/MODBUS-LIB/Modbus.d \
./Drivers/BSP/MODBUS-LIB/UARTCallback.d \
./Drivers/BSP/MODBUS-LIB/retarget.d 

OBJS += \
./Drivers/BSP/MODBUS-LIB/Encoder485.o \
./Drivers/BSP/MODBUS-LIB/EncoderMBTCP.o \
./Drivers/BSP/MODBUS-LIB/Modbus.o \
./Drivers/BSP/MODBUS-LIB/ModbusAbsoluteEncoder.o \
./Drivers/BSP/MODBUS-LIB/Sensor.o \
./Drivers/BSP/MODBUS-LIB/UARTCallback.o \
./Drivers/BSP/MODBUS-LIB/retarget.o 

CPP_DEPS += \
./Drivers/BSP/MODBUS-LIB/Encoder485.d \
./Drivers/BSP/MODBUS-LIB/EncoderMBTCP.d \
./Drivers/BSP/MODBUS-LIB/ModbusAbsoluteEncoder.d \
./Drivers/BSP/MODBUS-LIB/Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/MODBUS-LIB/%.o Drivers/BSP/MODBUS-LIB/%.su Drivers/BSP/MODBUS-LIB/%.cyclo: ../Drivers/BSP/MODBUS-LIB/%.cpp Drivers/BSP/MODBUS-LIB/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/river/st_ws/huiyang_motor_testing/Drivers/BSP/MODBUS-LIB" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/MODBUS-LIB/%.o Drivers/BSP/MODBUS-LIB/%.su Drivers/BSP/MODBUS-LIB/%.cyclo: ../Drivers/BSP/MODBUS-LIB/%.c Drivers/BSP/MODBUS-LIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/river/st_ws/huiyang_motor_testing/Drivers/BSP/MODBUS-LIB" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-MODBUS-2d-LIB

clean-Drivers-2f-BSP-2f-MODBUS-2d-LIB:
	-$(RM) ./Drivers/BSP/MODBUS-LIB/Encoder485.cyclo ./Drivers/BSP/MODBUS-LIB/Encoder485.d ./Drivers/BSP/MODBUS-LIB/Encoder485.o ./Drivers/BSP/MODBUS-LIB/Encoder485.su ./Drivers/BSP/MODBUS-LIB/EncoderMBTCP.cyclo ./Drivers/BSP/MODBUS-LIB/EncoderMBTCP.d ./Drivers/BSP/MODBUS-LIB/EncoderMBTCP.o ./Drivers/BSP/MODBUS-LIB/EncoderMBTCP.su ./Drivers/BSP/MODBUS-LIB/Modbus.cyclo ./Drivers/BSP/MODBUS-LIB/Modbus.d ./Drivers/BSP/MODBUS-LIB/Modbus.o ./Drivers/BSP/MODBUS-LIB/Modbus.su ./Drivers/BSP/MODBUS-LIB/ModbusAbsoluteEncoder.cyclo ./Drivers/BSP/MODBUS-LIB/ModbusAbsoluteEncoder.d ./Drivers/BSP/MODBUS-LIB/ModbusAbsoluteEncoder.o ./Drivers/BSP/MODBUS-LIB/ModbusAbsoluteEncoder.su ./Drivers/BSP/MODBUS-LIB/Sensor.cyclo ./Drivers/BSP/MODBUS-LIB/Sensor.d ./Drivers/BSP/MODBUS-LIB/Sensor.o ./Drivers/BSP/MODBUS-LIB/Sensor.su ./Drivers/BSP/MODBUS-LIB/UARTCallback.cyclo ./Drivers/BSP/MODBUS-LIB/UARTCallback.d ./Drivers/BSP/MODBUS-LIB/UARTCallback.o ./Drivers/BSP/MODBUS-LIB/UARTCallback.su ./Drivers/BSP/MODBUS-LIB/retarget.cyclo ./Drivers/BSP/MODBUS-LIB/retarget.d ./Drivers/BSP/MODBUS-LIB/retarget.o ./Drivers/BSP/MODBUS-LIB/retarget.su

.PHONY: clean-Drivers-2f-BSP-2f-MODBUS-2d-LIB

