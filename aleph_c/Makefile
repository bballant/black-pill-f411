NAME = aleph_c
TARGET = bin/$(NAME).bin

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
STFLASH	= $(shell which st-flash)

ARCH_FLAGS= -mthumb -mcpu=cortex-m4 $(FP_FLAGS)
ARCH_FLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDSCRIPT ?= stm32f411ceu6.ld
DEFS += -DSTM32F4

LIBS = -lm
LIBS += --static -nostartfiles
LIBS += -T$(LDSCRIPT)
LIBS += $(ARCH_FLAGS)
LIBS += -Wl,-Map=$(*).map
LIBS += -Wl,--gc-sections
LIBS += -specs=nosys.specs
LIBS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
LIBS += -Llibwwg -lwwg
LIBS += -Llibopencm3stm32/lib -lopencm3_stm32f4

CFLAGS = -Os -g -Wall -Wundef -std=c99
CFLAGS += $(ARCH_FLAGS)
CFLAGS += -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS += -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS += -fno-common -ffunction-sections -fdata-sections
CFLAGS += -I./rtos -I.
CFLAGS += -MD
CFLAGS += $(DEFS)
CFLAGS += -Ilibopencm3stm32/include
CFLAGS += -Ilibwwg/include

.PHONY: default all clean

default: $(TARGET)
all: default

RTOS = rtos/heap_4.o rtos/list.o rtos/port.o rtos/queue.o rtos/tasks.o rtos/opencm3.o
OBJECTS = src/main.o $(RTOS)

.PRECIOUS: $(TARGET) $(OBJECTS)

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.elf %.map: $(OBJECTS) $(LDSCRIPT)
	$(LD) $(OBJECTS) $(LIBS) -o $(*).elf
	$(SIZE) $(*).elf

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $< -o $@

# $(TARGET): $(OBJECTS)
# 	$(CC) $(OBJECTS) -Wall $(LIBS) -o $@

clean:
	-rm -f $(OBJECTS)
	-rm -f $(TARGET)
	-rm -rf bin/$(NAME).*

# Flash 64k Device
flash:	$(TARGET)
	$(STFLASH) $(FLASHSIZE) write $(TARGET) 0x8000000
