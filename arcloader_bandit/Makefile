ifeq ($(strip $(DEVKITPPC)),)
$(error "Set DEVKITPPC in your environment.")
endif

PREFIX = $(DEVKITPPC)/bin/powerpc-eabi-

CFLAGS = -mcpu=604 -m32 -mhard-float -mno-eabi -mno-sdata
CFLAGS += -ffreestanding -ffunction-sections -fdata-sections
CFLAGS += -I../baselibc/include
CFLAGS += -Wall -Wextra -Os
ASFLAGS =
LDFLAGS = -mcpu=604 -m32 -n -nostartfiles -nodefaultlibs -Wl,-gc-sections

CFLAGS += -mno-eabi -mno-sdata -O2 -ffreestanding
CFLAGS += -Wall -Wextra
DEFINES =  -DWITH_STDIO -DBASELIBC_INTERNAL
LDFLAGS += -nostartfiles -nodefaultlibs -L../baselibc

LDSCRIPT = of.ld
TARGET = boot

FILES = $(wildcard source/*.S) $(wildcard source/*.c)
OBJSx = $(FILES:source/%.S=build/%.o)
OBJS = $(OBJSx:source/%.c=build/%.o)
LIBS = -lcbase -lgcc

AR = $(PREFIX)ar
AS = $(PREFIX)as
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
LD = $(PREFIX)ld
OBJCOPY = $(PREFIX)objcopy
RANLIB = $(PREFIX)ranlib
STRIP = $(PREFIX)strip

HOSTCC = gcc

ifeq ($(NOMAPFILE),)
LDFLAGS += -Wl,-Map,$(TARGET).map
endif

ifneq ($(LDSCRIPT),)
LDFLAGS += -Wl,-T$(LDSCRIPT)
endif

DEPDIR = .deps

all: $(TARGET).xcf
	@[ -d $(DIR_BUILD) ] || mkdir $(DIR_BUILD)

$(TARGET).xcf: $(TARGET).elf hack-coff
	@echo "  OBJCOPY   $@"
	@rm -f $@
	@$(OBJCOPY) -O aixcoff-rs6000 -j .text -j .data -j .bss $< $@
	@./hack-coff $@

$(TARGET).elf: $(OBJS)
	@echo "  LINK      $@"
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@
	@cp $(TARGET).elf $(TARGET)_unstripped.elf
	@$(STRIP) $(TARGET).elf

hack-coff: hack-coff.c
	@echo "  HOSTCC    $@"
	@gcc -o $@ $<

ifneq ($(LDSCRIPT),)
$(TARGET).elf: $(LDSCRIPT)
endif

build/%.o: source/%.c
	@echo "  COMPILE   $<"
	@mkdir -p $(DEPDIR)
	@mkdir -p build
	@$(CC) $(CFLAGS) $(DEFINES) -Wp,-MMD,$(DEPDIR)/$(*F).d,-MQ,"$@",-MP -c $< -o $@

build/%.o: source/%.s
	@echo "  ASSEMBLE  $<"
	@mkdir -p build
	@$(CC) $(CFLAGS) $(DEFINES) $(ASFLAGS) -c $< -o $@

build/%.o: source/%.S
	@echo "  ASSEMBLE  $<"
	@mkdir -p build
	@$(CC) $(CFLAGS) $(DEFINES) $(ASFLAGS) -c $< -o $@

clean:
	rm -rf $(DEPDIR)
	rm -f $(TARGET) $(TARGET:%.elf=%_unstripped.elf) $(TARGET).map $(OBJS)

-include $(DEPDIR)/*

.PHONY: clean
