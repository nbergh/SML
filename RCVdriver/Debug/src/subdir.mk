################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Graphics.cpp \
../src/Input.cpp \
../src/PositionEstimation.cpp \
../src/UDPReceiver.cpp 

CU_SRCS += \
../src/LidarProcessing.cu \
../src/PathPlanning.cu \
../src/RCVdriver.cu 

CU_DEPS += \
./src/LidarProcessing.d \
./src/PathPlanning.d \
./src/RCVdriver.d 

OBJS += \
./src/Graphics.o \
./src/Input.o \
./src/LidarProcessing.o \
./src/PathPlanning.o \
./src/PositionEstimation.o \
./src/RCVdriver.o \
./src/UDPReceiver.o 

CPP_DEPS += \
./src/Graphics.d \
./src/Input.d \
./src/PositionEstimation.d \
./src/UDPReceiver.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 -gencode arch=compute_52,code=sm_52  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 --compile  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cu
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 -gencode arch=compute_52,code=sm_52  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -std=c++11 --compile --relocatable-device-code=false -gencode arch=compute_52,code=sm_52  -x cu -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


