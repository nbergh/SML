################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/GPSprocessing.cpp \
../src/Graphics.cpp \
../src/Input.cpp 

CU_SRCS += \
../src/CudaErrorCheckFunctions.cu \
../src/PathPlanning.cu \
../src/RCVdriver.cu 

CU_DEPS += \
./src/CudaErrorCheckFunctions.d \
./src/PathPlanning.d \
./src/RCVdriver.d 

OBJS += \
./src/CudaErrorCheckFunctions.o \
./src/GPSprocessing.o \
./src/Graphics.o \
./src/Input.o \
./src/PathPlanning.o \
./src/RCVdriver.o 

CPP_DEPS += \
./src/GPSprocessing.d \
./src/Graphics.d \
./src/Input.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cu
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -gencode arch=compute_30,code=sm_30  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 --compile --relocatable-device-code=false -gencode arch=compute_30,code=compute_30 -gencode arch=compute_30,code=sm_30  -x cu -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 -gencode arch=compute_30,code=sm_30  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.5/bin/nvcc -G -g -O0 --compile  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


