################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/bmi160.c \
../source/bmm150.c \
../source/i2c.c \
../source/lcd.c \
../source/main.c \
../source/sensors.c 

C_DEPS += \
./source/bmi160.d \
./source/bmm150.d \
./source/i2c.d \
./source/lcd.d \
./source/main.d \
./source/sensors.d 

OBJS += \
./source/bmi160.o \
./source/bmm150.o \
./source/i2c.o \
./source/lcd.o \
./source/main.o \
./source/sensors.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-buildroot-linux-uclibcgnueabihf-gcc -I/home/ubuntu/Documents/buildroot-2020.02.8/output/host/usr/include -O0 -g3 -Wall -c -fmessage-length=0 -pthread -lrt -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/bmi160.d ./source/bmi160.o ./source/bmm150.d ./source/bmm150.o ./source/i2c.d ./source/i2c.o ./source/lcd.d ./source/lcd.o ./source/main.d ./source/main.o ./source/sensors.d ./source/sensors.o

.PHONY: clean-source

