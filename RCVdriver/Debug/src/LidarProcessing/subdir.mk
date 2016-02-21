################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/LidarProcessing/LidarUDPReceiver.cpp 

CU_SRCS += \
../src/LidarProcessing/LidarProcessing.cu 

CU_DEPS += \
./src/LidarProcessing/LidarProcessing.d 

OBJS += \
./src/LidarProcessing/LidarProcessing.o \
./src/LidarProcessing/LidarUDPReceiver.o 

CPP_DEPS += \
./src/LidarProcessing/LidarUDPReceiver.d 


# Each subdirectory must supply rules for building sources it contributes
src/LidarProcessing/%.o: ../src/LidarProcessing/%.cu
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 -gencode arch=compute_52,code=sm_52  -odir "src/LidarProcessing" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 --compile --relocatable-device-code=false -gencode arch=compute_52,code=sm_52  -x cu -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/LidarProcessing/%.o: ../src/LidarProcessing/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 -gencode arch=compute_52,code=sm_52  -odir "src/LidarProcessing" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 --compile  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


