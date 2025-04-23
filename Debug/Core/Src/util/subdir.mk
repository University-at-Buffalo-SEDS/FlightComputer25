################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/util/avghistory.c \
../Core/Src/util/matrix.c \
../Core/Src/util/ring_buffer.c 

OBJS += \
./Core/Src/util/avghistory.o \
./Core/Src/util/matrix.o \
./Core/Src/util/ring_buffer.o 

C_DEPS += \
./Core/Src/util/avghistory.d \
./Core/Src/util/matrix.d \
./Core/Src/util/ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/util/%.o Core/Src/util/%.su Core/Src/util/%.cyclo: ../Core/Src/util/%.c Core/Src/util/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-util

clean-Core-2f-Src-2f-util:
	-$(RM) ./Core/Src/util/avghistory.cyclo ./Core/Src/util/avghistory.d ./Core/Src/util/avghistory.o ./Core/Src/util/avghistory.su ./Core/Src/util/matrix.cyclo ./Core/Src/util/matrix.d ./Core/Src/util/matrix.o ./Core/Src/util/matrix.su ./Core/Src/util/ring_buffer.cyclo ./Core/Src/util/ring_buffer.d ./Core/Src/util/ring_buffer.o ./Core/Src/util/ring_buffer.su

.PHONY: clean-Core-2f-Src-2f-util

