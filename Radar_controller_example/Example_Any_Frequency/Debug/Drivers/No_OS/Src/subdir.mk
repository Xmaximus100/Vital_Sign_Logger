################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/No_OS/Src/no_os_alloc.c \
../Drivers/No_OS/Src/no_os_util.c 

OBJS += \
./Drivers/No_OS/Src/no_os_alloc.o \
./Drivers/No_OS/Src/no_os_util.o 

C_DEPS += \
./Drivers/No_OS/Src/no_os_alloc.d \
./Drivers/No_OS/Src/no_os_util.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/No_OS/Src/%.o Drivers/No_OS/Src/%.su Drivers/No_OS/Src/%.cyclo: ../Drivers/No_OS/Src/%.c Drivers/No_OS/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/patryk.celebanski/Documents/Studia/Masters Dissertation/STM32Project/Vital_Sign_Logger/Radar_controller_example/Example_Any_Frequency/Drivers/ADF5355/Inc" -I"C:/Users/patryk.celebanski/Documents/Studia/Masters Dissertation/STM32Project/Vital_Sign_Logger/Radar_controller_example/Example_Any_Frequency/Drivers/No_OS/Inc" -I"C:/Users/patryk.celebanski/Documents/Studia/Masters Dissertation/STM32Project/Vital_Sign_Logger/Radar_controller_example/Example_Any_Frequency/Drivers/AD7676/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-No_OS-2f-Src

clean-Drivers-2f-No_OS-2f-Src:
	-$(RM) ./Drivers/No_OS/Src/no_os_alloc.cyclo ./Drivers/No_OS/Src/no_os_alloc.d ./Drivers/No_OS/Src/no_os_alloc.o ./Drivers/No_OS/Src/no_os_alloc.su ./Drivers/No_OS/Src/no_os_util.cyclo ./Drivers/No_OS/Src/no_os_util.d ./Drivers/No_OS/Src/no_os_util.o ./Drivers/No_OS/Src/no_os_util.su

.PHONY: clean-Drivers-2f-No_OS-2f-Src

