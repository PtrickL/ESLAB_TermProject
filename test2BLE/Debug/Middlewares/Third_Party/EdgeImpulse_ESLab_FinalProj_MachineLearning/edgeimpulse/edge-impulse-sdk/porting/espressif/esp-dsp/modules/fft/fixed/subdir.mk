################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.c 

C_DEPS += \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.d 

OBJS += \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/%.o Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/%.su Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/%.cyclo: ../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/%.c Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../BlueNRG_MS/App -I../BlueNRG_MS/Target -I../Drivers/BSP/B-L475E-IOT01A1 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/BlueNRG-MS/utils -I../Middlewares/ST/BlueNRG-MS/includes -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Drivers/BSP/Components -I../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/ -I"C:/Users/tingx/OneDrive/Documents/NTU/EmbedSysLab/ESLAB_TermProject/test2BLE/Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/CMSIS/DSP/Include/dsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-porting-2f-espressif-2f-esp-2d-dsp-2f-modules-2f-fft-2f-fixed

clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-porting-2f-espressif-2f-esp-2d-dsp-2f-modules-2f-fft-2f-fixed:
	-$(RM) ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.cyclo ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.d ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.o ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-porting-2f-espressif-2f-esp-2d-dsp-2f-modules-2f-fft-2f-fixed

