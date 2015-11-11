################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/PositionEstimation/GPSdataReceiver.cpp \
../src/PositionEstimation/PositionEstimation.cpp 

OBJS += \
./src/PositionEstimation/GPSdataReceiver.o \
./src/PositionEstimation/PositionEstimation.o 

CPP_DEPS += \
./src/PositionEstimation/GPSdataReceiver.d \
./src/PositionEstimation/PositionEstimation.d 


# Each subdirectory must supply rules for building sources it contributes
src/PositionEstimation/%.o: ../src/PositionEstimation/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -gencode arch=compute_30,code=sm_30  -odir "src/PositionEstimation" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 --compile  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


