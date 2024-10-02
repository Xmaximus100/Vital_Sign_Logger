################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ADF5355/Src/adf5355.c \
../Drivers/ADF5355/Src/basic_example.c 

OBJS += \
./Drivers/ADF5355/Src/adf5355.o \
./Drivers/ADF5355/Src/basic_example.o 

C_DEPS += \
./Drivers/ADF5355/Src/adf5355.d \
./Drivers/ADF5355/Src/basic_example.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ADF5355/Src/%.o Drivers/ADF5355/Src/%.su Drivers/ADF5355/Src/%.cyclo: ../Drivers/ADF5355/Src/%.c Drivers/ADF5355/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/patryk.celebanski/Documents/Studia/Masters Dissertation/STM32Project/Vital_Sign_Logger/Radar_controller_example/Example_Any_Frequency/Drivers/ADF5355/Inc" -I"C:/Users/patryk.celebanski/Documents/Studia/Masters Dissertation/STM32Project/Vital_Sign_Logger/Radar_controller_example/Example_Any_Frequency/Drivers/No_OS/Inc" -I"C:/Users/patryk.celebanski/Documents/Studia/Masters Dissertation/STM32Project/Vital_Sign_Logger/Radar_controller_example/Example_Any_Frequency/Drivers/AD7676/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ADF5355-2f-Src

clean-Drivers-2f-ADF5355-2f-Src:
	-$(RM) ./Drivers/ADF5355/Src/adf5355.cyclo ./Drivers/ADF5355/Src/adf5355.d ./Drivers/ADF5355/Src/adf5355.o ./Drivers/ADF5355/Src/adf5355.su ./Drivers/ADF5355/Src/basic_example.cyclo ./Drivers/ADF5355/Src/basic_example.d ./Drivers/ADF5355/Src/basic_example.o ./Drivers/ADF5355/Src/basic_example.su

.PHONY: clean-Drivers-2f-ADF5355-2f-Src

