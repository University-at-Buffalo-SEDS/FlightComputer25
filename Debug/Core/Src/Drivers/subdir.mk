################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/BMI088.c \
../Core/Src/Drivers/BMP390.c \
../Core/Src/Drivers/flash.c 

OBJS += \
./Core/Src/Drivers/BMI088.o \
./Core/Src/Drivers/BMP390.o \
./Core/Src/Drivers/flash.o 

C_DEPS += \
./Core/Src/Drivers/BMI088.d \
./Core/Src/Drivers/BMP390.d \
./Core/Src/Drivers/flash.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/BMI088.cyclo ./Core/Src/Drivers/BMI088.d ./Core/Src/Drivers/BMI088.o ./Core/Src/Drivers/BMI088.su ./Core/Src/Drivers/BMP390.cyclo ./Core/Src/Drivers/BMP390.d ./Core/Src/Drivers/BMP390.o ./Core/Src/Drivers/BMP390.su ./Core/Src/Drivers/flash.cyclo ./Core/Src/Drivers/flash.d ./Core/Src/Drivers/flash.o ./Core/Src/Drivers/flash.su

.PHONY: clean-Core-2f-Src-2f-Drivers

