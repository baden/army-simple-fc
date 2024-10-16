# ===================================================================================
# Project:  CH32V003 Tiny Arkanoid
# Author:   Stefan Wagner
# Year:     2023
# URL:      https://github.com/wagiminator
# ===================================================================================
# Type "make help" in the command line.
# ===================================================================================

# Files and Folders
MAINFILE = src/main.c
TARGET   = army-simple-fc
INCLUDE  = include
TOOLS    = tools

# Microcontroller Settings
# F_CPU    = 12000000
#F_CPU    = 24000000
F_CPU    = 3000000
LDSCRIPT = linker/ch32v003.ld
CPUARCH  = -march=rv32ec -mabi=ilp32e

# Toolchain
#PREFIX   = riscv64-unknown-elf
PREFIX   = /opt/homebrew/bin/riscv64-unknown-elf
#PREFIX   = /Users/baden/.local/opt/xpack-riscv-none-elf-gcc-12.2.0-3/bin/riscv-none-elf
CC       = $(PREFIX)-gcc
OBJCOPY  = $(PREFIX)-objcopy
OBJDUMP  = $(PREFIX)-objdump
OBJSIZE  = $(PREFIX)-size
NEWLIB   = /usr/include/newlib
ISPTOOL  = python3 $(TOOLS)/rvprog.py -f $(TARGET).bin
CLEAN    = rm -f *.lst *.obj *.cof *.list *.map *.eep.hex *.o *.d

# Compiler Flags
CFLAGS   = $(CPUARCH) -DF_CPU=$(F_CPU) -I$(NEWLIB) -I$(INCLUDE) -I. -L$(INCLUDE)
CFLAGS  += -g -Os -flto -ffunction-sections -fno-builtin -Wall
LDFLAGS  = -T$(LDSCRIPT) -nostdlib -lgcc -static-libgcc -Wl,--gc-sections
CFILES   = $(MAINFILE) src/crsf.c src/dac.c $(wildcard $(INCLUDE)/*.c) $(wildcard $(INCLUDE)/*.S)

# Symbolic Targets
help:
	@echo "Use the following commands:"
	@echo "make all       compile and build $(TARGET).elf/.bin/.hex/.asm"
	@echo "make hex       compile and build $(TARGET).hex"
	@echo "make asm       compile and disassemble to $(TARGET).asm"
	@echo "make bin       compile and build $(TARGET).bin"
	@echo "make flash     compile and upload to MCU"
	@echo "make clean     remove all build files"

$(TARGET)-L.elf: $(CFILES)
	@echo "Building $(TARGET).elf ..."
	@$(CC) -o $@ $^ $(CFLAGS) -DDEVICE_MODE=2 $(LDFLAGS)

$(TARGET)-R.elf: $(CFILES)
	@echo "Building $(TARGET).elf ..."
	@$(CC) -o $@ $^ $(CFLAGS) -DDEVICE_MODE=3 $(LDFLAGS)

$(TARGET)-UD.elf: $(CFILES)
	@echo "Building $(TARGET).elf ..."
	@$(CC) -o $@ $^ $(CFLAGS) -DDEVICE_MODE=1 $(LDFLAGS)

$(TARGET).lst: $(TARGET).elf
	@echo "Building $(TARGET).lst ..."
	@$(OBJDUMP) -S $^ > $(TARGET).lst

$(TARGET).map: $(TARGET).elf
	@echo "Building $(TARGET).map ..."
	@$(OBJDUMP) -t $^ > $(TARGET).map

$(TARGET).bin: $(TARGET).elf
	@echo "Building $(TARGET).bin ..."
	@$(OBJCOPY) -O binary $< $(TARGET).bin

$(TARGET)-L.hex: $(TARGET)-L.elf
	@echo "Building $(TARGET).hex ..."
	@$(OBJCOPY) -O ihex $< $(TARGET)-L.hex

$(TARGET)-R.hex: $(TARGET)-R.elf
	@echo "Building $(TARGET).hex ..."
	@$(OBJCOPY) -O ihex $< $(TARGET)-R.hex

$(TARGET)-UD.hex: $(TARGET)-UD.elf
	@echo "Building $(TARGET).hex ..."
	@$(OBJCOPY) -O ihex $< $(TARGET)-UD.hex

$(TARGET).asm: $(TARGET).elf
	@echo "Disassembling to $(TARGET).asm ..."
	@$(OBJDUMP) -d $(TARGET).elf > $(TARGET).asm

#all:	$(TARGET).lst $(TARGET).map $(TARGET).bin $(TARGET)-L.hex $(TARGET)-R.hex $(TARGET)-UD.hex $(TARGET).asm size
all:	$(TARGET)-L.hex $(TARGET)-R.hex $(TARGET)-UD.hex size

elf:	$(TARGET).elf removetemp size

bin:	$(TARGET).bin removetemp size removeelf

hex:	$(TARGET)-R.hex $(TARGET)-L.hex $(TARGET)-UD.hex removetemp size removeelf

asm:	$(TARGET).asm removetemp size removeelf

flash:	$(TARGET).bin size removeelf
	@echo "Uploading to MCU ..."
	@$(ISPTOOL)

clean:
	@echo "Cleaning all up ..."
	@$(CLEAN)
	@rm -f $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).asm

size:
	@echo "------------------"
	@echo "FLASH: $(shell $(OBJSIZE) -d $(TARGET).elf | awk '/[0-9]/ {print $$1 + $$2}') bytes"
	@echo "SRAM:  $(shell $(OBJSIZE) -d $(TARGET).elf | awk '/[0-9]/ {print $$2 + $$3}') bytes"
	@echo "------------------"

removetemp:
	@echo "Removing temporary files ..."
	@$(CLEAN)

removeelf:
	@echo "Removing $(TARGET).elf ..."
	@rm -f $(TARGET).elf
