################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.cpp \
../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.cpp \
../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.cpp \
../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_utils.cpp 

OBJS += \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.o \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.o \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.o \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_utils.o 

CPP_DEPS += \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.d \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.d \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.d \
./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/%.o Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/%.su Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/%.cyclo: ../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/%.cpp Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../BlueNRG_MS/App -I../BlueNRG_MS/Target -I../Drivers/BSP/B-L475E-IOT01A1 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/BlueNRG-MS/utils -I../Middlewares/ST/BlueNRG-MS/includes -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -I../Drivers/BSP/Components -I../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/ -I../Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-tensorflow-2f-lite-2f-kernels-2f-internal

clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-tensorflow-2f-lite-2f-kernels-2f-internal:
	-$(RM) ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.cyclo ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.d ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.o ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/portable_tensor_utils.su ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.cyclo ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.d ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.o ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/quantization_util.su ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.cyclo ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.d ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.o ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/reference_portable_tensor_utils.su ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_utils.cyclo ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_utils.d ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_utils.o ./Middlewares/Third_Party/EdgeImpulse_ESLab_FinalProj_MachineLearning/edgeimpulse/edge-impulse-sdk/tensorflow/lite/kernels/internal/tensor_utils.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-EdgeImpulse_ESLab_FinalProj_MachineLearning-2f-edgeimpulse-2f-edge-2d-impulse-2d-sdk-2f-tensorflow-2f-lite-2f-kernels-2f-internal

