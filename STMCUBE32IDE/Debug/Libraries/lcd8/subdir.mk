################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/lcd8/i2c.c 

OBJS += \
./Libraries/lcd8/i2c.o 

C_DEPS += \
./Libraries/lcd8/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/lcd8/%.o Libraries/lcd8/%.su Libraries/lcd8/%.cyclo: ../Libraries/lcd8/%.c Libraries/lcd8/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103x6 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/workspace_1.14.1/mojProjekat/Libraries/lcd8" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Libraries-2f-lcd8

clean-Libraries-2f-lcd8:
	-$(RM) ./Libraries/lcd8/i2c.cyclo ./Libraries/lcd8/i2c.d ./Libraries/lcd8/i2c.o ./Libraries/lcd8/i2c.su

.PHONY: clean-Libraries-2f-lcd8

