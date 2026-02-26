################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/coil_calib.c \
../Core/Src/coil_driver.c \
../Core/Src/debug_console.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/levitation_control.c \
../Core/Src/main.c \
../Core/Src/qspi_flash.c \
../Core/Src/quadspi.c \
../Core/Src/sensor_mlx90393.c \
../Core/Src/spi.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/usbd_hid_if.c 

OBJS += \
./Core/Src/coil_calib.o \
./Core/Src/coil_driver.o \
./Core/Src/debug_console.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/levitation_control.o \
./Core/Src/main.o \
./Core/Src/qspi_flash.o \
./Core/Src/quadspi.o \
./Core/Src/sensor_mlx90393.o \
./Core/Src/spi.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/usbd_hid_if.o 

C_DEPS += \
./Core/Src/coil_calib.d \
./Core/Src/coil_driver.d \
./Core/Src/debug_console.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/levitation_control.d \
./Core/Src/main.d \
./Core/Src/qspi_flash.d \
./Core/Src/quadspi.d \
./Core/Src/sensor_mlx90393.d \
./Core/Src/spi.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/usbd_hid_if.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H750xx -c -I../Core/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/coil_calib.cyclo ./Core/Src/coil_calib.d ./Core/Src/coil_calib.o ./Core/Src/coil_calib.su ./Core/Src/coil_driver.cyclo ./Core/Src/coil_driver.d ./Core/Src/coil_driver.o ./Core/Src/coil_driver.su ./Core/Src/debug_console.cyclo ./Core/Src/debug_console.d ./Core/Src/debug_console.o ./Core/Src/debug_console.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/levitation_control.cyclo ./Core/Src/levitation_control.d ./Core/Src/levitation_control.o ./Core/Src/levitation_control.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/qspi_flash.cyclo ./Core/Src/qspi_flash.d ./Core/Src/qspi_flash.o ./Core/Src/qspi_flash.su ./Core/Src/quadspi.cyclo ./Core/Src/quadspi.d ./Core/Src/quadspi.o ./Core/Src/quadspi.su ./Core/Src/sensor_mlx90393.cyclo ./Core/Src/sensor_mlx90393.d ./Core/Src/sensor_mlx90393.o ./Core/Src/sensor_mlx90393.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usbd_hid_if.cyclo ./Core/Src/usbd_hid_if.d ./Core/Src/usbd_hid_if.o ./Core/Src/usbd_hid_if.su

.PHONY: clean-Core-2f-Src

