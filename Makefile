CC := arm-none-eabi-gcc
CXX := arm-none-eabi-g++
LD := arm-none-eabi-g++
AR := arm-none-eabi-ar
OBJCOPY := arm-none-eabi-objcopy
RM := rm
LOADER := ../RFduino/RFDLoader_osx
SIZE := arm-none-eabi-size

PROGRAM := alfred

MCU := cortex-m0
F_CPU := 16000000
#SERIAL_PORT := /dev/cu.usbserial-DN00CYBM 
SERIAL_PORT := /dev/cu.usbserial-DA01LUCG 

INCLUDES := -Iinclude -Iinclude/RFduino -Iinclude/RFduino/include -Iinclude/CMSIS/CMSIS/include
LIB_DIR := -Llib 

CORE := core/core.a
LSCRIPT := RFduino.ld
LIBS := -lRFduinoSystem -lRFduino -lRFduinoBLE -lRFduinoGZLL

CFLAGS := -c -g -Os -w -ffunction-sections -fdata-sections -fno-builtin -MMD -mcpu=$(MCU) -DF_CPU=$(F_CPU) -mthumb -D__RFduino__ -DARDUINO=100
CXXFLAGS := -c -g -Os -w -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-builtin -MMD -mcpu=$(MCU) -DF_CPU=$(F_CPU) -mthumb -D__RFduino__ -DARDUINO=100
LDFLAGS := -Wl,--gc-sections --specs=nosys.specs -mcpu=$(MCU) -mthumb -D__RFduino__ -Wl,-Map,$(PROGRAM).map -Wl,--cref -Wl,--warn-common -Wl,--warn-section-align
ASFLAGS := -c -g -assembler-with-cpp
ARFLAGS := rcs
OCFLAGS := -O ihex

CORE_C_SRCS := $(wildcard core/*.c)
CORE_CXX_SRCS := $(wildcard core/*.cpp)

CORE_OBJS := $(patsubst %.c,%.o,$(CORE_C_SRCS))
CORE_OBJS += $(patsubst %.cpp,%.o,$(CORE_CXX_SRCS))

C_SRCS := $(wildcard src/*.c)
CXX_SRCS := $(wildcard src/*.cpp)

OBJS := $(patsubst %.c,%.o,$(C_SRCS))
OBJS += $(patsubst %.cpp,%.o,$(CXX_SRCS))

$(PROGRAM).hex: $(PROGRAM).elf
	$(OBJCOPY) $(OCFLAGS) $< $@
	$(SIZE) -B $(PROGRAM).elf

$(PROGRAM).elf: $(CORE) $(OBJS) $(LSCRIPT)
	$(LD) $(LDFLAGS) -T$(LSCRIPT) $(LIB_DIR) $(OBJS) $< $(LIBS) -o $@

$(CORE): $(CORE_OBJS)
	$(AR) $(ARFLAGS) $@ $(CORE_OBJS)

%.o: %.c Makefile
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.cpp Makefile
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

.PHONY: clean upload
clean:
	$(RM) -rf $(CORE_OBJS) $(OBJS) $(CORE) $(PROGRAM).map $(PROGRAM).elf $(PROGRAM).hex src/*.d core/*.d 

upload:
	$(LOADER) $(SERIAL_PORT) $(PROGRAM).hex 
