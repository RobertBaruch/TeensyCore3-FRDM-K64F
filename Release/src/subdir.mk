################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/analog.c \
../src/eeprom.c \
../src/keylayouts.c \
../src/math_helper.c \
../src/mk20dx128.c \
../src/nonstd.c \
../src/pins_teensy.c \
../src/ser_print.c \
../src/serial1.c \
../src/serial2.c \
../src/serial3.c \
../src/serial4.c \
../src/serial5.c \
../src/serial6.c \
../src/serial6_lpuart.c \
../src/touch.c \
../src/usb_desc.c \
../src/usb_dev.c \
../src/usb_joystick.c \
../src/usb_keyboard.c \
../src/usb_mem.c \
../src/usb_midi.c \
../src/usb_mouse.c \
../src/usb_mtp.c \
../src/usb_rawhid.c \
../src/usb_seremu.c \
../src/usb_serial.c \
../src/usb_touch.c 

CPP_SRCS += \
../src/AudioStream.cpp \
../src/DMAChannel.cpp \
../src/HardwareSerial1.cpp \
../src/HardwareSerial2.cpp \
../src/HardwareSerial3.cpp \
../src/HardwareSerial4.cpp \
../src/HardwareSerial5.cpp \
../src/HardwareSerial6.cpp \
../src/IPAddress.cpp \
../src/IntervalTimer.cpp \
../src/Print.cpp \
../src/Stream.cpp \
../src/Tone.cpp \
../src/WMath.cpp \
../src/WString.cpp \
../src/avr_emulation.cpp \
../src/new.cpp \
../src/usb_audio.cpp \
../src/usb_flightsim.cpp \
../src/usb_inst.cpp \
../src/yield.cpp 

S_UPPER_SRCS += \
../src/memcpy-armv7m.S \
../src/memset.S 

OBJS += \
./src/AudioStream.o \
./src/DMAChannel.o \
./src/HardwareSerial1.o \
./src/HardwareSerial2.o \
./src/HardwareSerial3.o \
./src/HardwareSerial4.o \
./src/HardwareSerial5.o \
./src/HardwareSerial6.o \
./src/IPAddress.o \
./src/IntervalTimer.o \
./src/Print.o \
./src/Stream.o \
./src/Tone.o \
./src/WMath.o \
./src/WString.o \
./src/analog.o \
./src/avr_emulation.o \
./src/eeprom.o \
./src/keylayouts.o \
./src/math_helper.o \
./src/memcpy-armv7m.o \
./src/memset.o \
./src/mk20dx128.o \
./src/new.o \
./src/nonstd.o \
./src/pins_teensy.o \
./src/ser_print.o \
./src/serial1.o \
./src/serial2.o \
./src/serial3.o \
./src/serial4.o \
./src/serial5.o \
./src/serial6.o \
./src/serial6_lpuart.o \
./src/touch.o \
./src/usb_audio.o \
./src/usb_desc.o \
./src/usb_dev.o \
./src/usb_flightsim.o \
./src/usb_inst.o \
./src/usb_joystick.o \
./src/usb_keyboard.o \
./src/usb_mem.o \
./src/usb_midi.o \
./src/usb_mouse.o \
./src/usb_mtp.o \
./src/usb_rawhid.o \
./src/usb_seremu.o \
./src/usb_serial.o \
./src/usb_touch.o \
./src/yield.o 

S_UPPER_DEPS += \
./src/memcpy-armv7m.d \
./src/memset.d 

C_DEPS += \
./src/analog.d \
./src/eeprom.d \
./src/keylayouts.d \
./src/math_helper.d \
./src/mk20dx128.d \
./src/nonstd.d \
./src/pins_teensy.d \
./src/ser_print.d \
./src/serial1.d \
./src/serial2.d \
./src/serial3.d \
./src/serial4.d \
./src/serial5.d \
./src/serial6.d \
./src/serial6_lpuart.d \
./src/touch.d \
./src/usb_desc.d \
./src/usb_dev.d \
./src/usb_joystick.d \
./src/usb_keyboard.d \
./src/usb_mem.d \
./src/usb_midi.d \
./src/usb_mouse.d \
./src/usb_mtp.d \
./src/usb_rawhid.d \
./src/usb_seremu.d \
./src/usb_serial.d \
./src/usb_touch.d 

CPP_DEPS += \
./src/AudioStream.d \
./src/DMAChannel.d \
./src/HardwareSerial1.d \
./src/HardwareSerial2.d \
./src/HardwareSerial3.d \
./src/HardwareSerial4.d \
./src/HardwareSerial5.d \
./src/HardwareSerial6.d \
./src/IPAddress.d \
./src/IntervalTimer.d \
./src/Print.d \
./src/Stream.d \
./src/Tone.d \
./src/WMath.d \
./src/WString.d \
./src/avr_emulation.d \
./src/new.d \
./src/usb_audio.d \
./src/usb_flightsim.d \
./src/usb_inst.d \
./src/yield.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DF_CPU=120000000 -D__FRDM_K64F__ -DUSB_SERIAL -DLAYOUT_US_ENGLISH -I"F:\kalkulyator-workspace-v2\TeensyCore3\src" -std=gnu++11 -fabi-version=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DF_CPU=120000000 -D__FRDM_K64F__ -DUSB_SERIAL -DLAYOUT_US_ENGLISH -I"F:\kalkulyator-workspace-v2\TeensyCore3\src" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -x assembler-with-cpp -DF_CPU=120000000 -DLAYOUT_US_ENGLISH -DUSB_SERIAL -D__FRDM_K64F__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


