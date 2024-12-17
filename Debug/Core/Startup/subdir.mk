################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f767zitx.s 

OBJS += \
./Core/Startup/startup_stm32f767zitx.o 

S_DEPS += \
./Core/Startup/startup_stm32f767zitx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I"C:/Users/achapin/STM32CubeIDE/workspace_1.17.0/F767ZIT6_Screen_Test/FT812" -I"C:/Users/achapin/STM32CubeIDE/workspace_1.17.0/F767ZIT6_Screen_Test/lvgl" -I"C:/Users/achapin/STM32CubeIDE/workspace_1.17.0/F767ZIT6_Screen_Test/Riverdi_EVE" -I"C:/Users/achapin/STM32CubeIDE/workspace_1.17.0/F767ZIT6_Screen_Test/lvgl/src" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f767zitx.d ./Core/Startup/startup_stm32f767zitx.o

.PHONY: clean-Core-2f-Startup

