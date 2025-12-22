################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/debug_log.cpp \
../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/ei_classifier_porting.cpp 

OBJS += \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/debug_log.o \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/ei_classifier_porting.o 

CPP_DEPS += \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/debug_log.d \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/ei_classifier_porting.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/%.o Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/%.su Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/%.cyclo: ../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/%.cpp Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../BlueNRG_MS/App -I../BlueNRG_MS/Target -I../Drivers/BSP/B-L475E-IOT01A1 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/BlueNRG-MS/utils -I../Middlewares/ST/BlueNRG-MS/includes -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Drivers/BSP/Components -I../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/ -I../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-porting-2f-particle

clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-porting-2f-particle:
	-$(RM) ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/debug_log.cyclo ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/debug_log.d ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/debug_log.o ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/debug_log.su ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/ei_classifier_porting.cyclo ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/ei_classifier_porting.d ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/ei_classifier_porting.o ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/porting/particle/ei_classifier_porting.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-porting-2f-particle

