################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f411xx_gpio_driver.c \
../drivers/Src/stm32f411xx_i2c_driver.c \
../drivers/Src/stm32f411xx_spi_drivers.c 

OBJS += \
./drivers/Src/stm32f411xx_gpio_driver.o \
./drivers/Src/stm32f411xx_i2c_driver.o \
./drivers/Src/stm32f411xx_spi_drivers.o 

C_DEPS += \
./drivers/Src/stm32f411xx_gpio_driver.d \
./drivers/Src/stm32f411xx_i2c_driver.d \
./drivers/Src/stm32f411xx_spi_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I"C:/EMBEDDED C/My_Workspace_course_3/stm32f4xx_drivers.h/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f411xx_gpio_driver.d ./drivers/Src/stm32f411xx_gpio_driver.o ./drivers/Src/stm32f411xx_gpio_driver.su ./drivers/Src/stm32f411xx_i2c_driver.d ./drivers/Src/stm32f411xx_i2c_driver.o ./drivers/Src/stm32f411xx_i2c_driver.su ./drivers/Src/stm32f411xx_spi_drivers.d ./drivers/Src/stm32f411xx_spi_drivers.o ./drivers/Src/stm32f411xx_spi_drivers.su

.PHONY: clean-drivers-2f-Src

