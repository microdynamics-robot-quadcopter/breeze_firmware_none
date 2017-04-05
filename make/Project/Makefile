PROJECT := microdynamics_stm32_template

MCU     := cortex-m3

DDEFS   += -DSTM32F10X_MD
DDEFS   += -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

SRC_ASM := ../Libraries/CMSIS/startup/gcc/startup_stm32f10x_md.s

SRC_C   += $(wildcard ../Libraries/CMSIS/*.c)
SRC_C   += $(wildcard ../Libraries/FWLib/src/*.c)
SRC_C   += $(wildcard ../Drivers/Driver_Delay/*.c)
# SRC_C   += $(wildcard ../Drivers/Driver_USART/*.c)
SRC_C   += $(wildcard ../Modules/Module_Led/*.c)
SRC_C   += $(wildcard ../User/*.c)

INCDIR  += -I../Libraries/CMSIS/ \
           -I../Libraries/FWLib/inc/ \
           -I../Drivers/Driver_Delay/ \
           -I../Drivers/Driver_IO/ \
           -I../Drivers/Driver_USART/ \
           -I../Modules/Module_Led/ \
           -I../User/ \

LINK_SCRIPT := ../Libraries/LinkScript/stm32f10x_flash.ld

CC_PREFIX := arm-none-eabi-

CC   := $(CC_PREFIX)gcc
CXX  := $(CC_PREFIX)g++
CP   := $(CC_PREFIX)objcopy
GDB  := $(CC_PREFIX)gdb
SIZE := $(CC_PREFIX)size
AS   := $(CC) -x assembler-with-cpp
HEX  := $(CP) -O ihex
BIN  := $(CP) -O binary -S

OPT  += -Os
OPT  += -fsingle-precision-constant
OPT  += -fno-common

DEFS := $(DDEFS) -DRUN_FROM_FLASH=1

FLAGS_MCU := -mcpu=$(MCU)
FLAGS_AS  := $(FLAGS_MCU) $(OPT) -g -gdwarf-2 -mthumb
FLAGS_C   := $(FLAGS_MCU) $(OPT) -g -gdwarf-2 -mthumb -fomit-frame-pointer \
             -Wall -fverbose-asm $(DEFS)
FLAGS_CXX := $(FLAGS_MCU) $(OPT) -g -gdwarf-2 -mthumb -std=c++11 \
             -fomit-frame-pointer -Wall -fverbose-asm -fno-exceptions \
             -fno-rtti -fno-threadsafe-statics -fvisibility=hidden \
             $(DEFS)
FLAGS_LD  := $(FLAGS_MCU) $(OPT) -lm -g -gdwarf-2 -mthumb -nostartfiles \
             -Xlinker --gc-sections -T$(LINK_SCRIPT) \
             -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch

OBJS      := $(filter %.o, $(SRC_ASM:.s=.o)) $(filter %.o, $(SRC_C:.c=.o))

TYPE_BURN  := openocd_swd_flash
TYPE_DEBUG := openocd_swd_debug
TYPE_ERASE := openocd_swd_erase

###############################################################################

.PHONY: all burn debug erase clean

all: $(OBJS) $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin
	$(SIZE) $(PROJECT).elf

%.o: %.c
	$(CC) -c $(FLAGS_C) $(INCDIR) $< -o $@

%.o: %.s
	$(AS) -c $(FLAGS_AS) $< -o $@

%.elf: $(OBJS)
	$(CC) $(OBJS) $(FLAGS_LD) -o $@

%.hex: %.elf
	$(HEX) $< $@

%.bin: %.elf
	$(BIN) $< $@

###############################################################################

burn:  $(TYPE_BURN)
debug: $(TYPE_DEBUG)
erase: $(TYPE_ERASE)

openocd_swd_flash: $(PROJECT).bin
	openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "init" -c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase $(PROJECT).bin 0x08000000" -c "sleep 100" -c "verify_image $(PROJECT).bin 0x08000000" -c "sleep 100" -c "reset run" -c shutdown

openocd_swd_debug: $(PROJECT).bin
	xterm -e openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "init" -c "halt" -c "reset halt" &
	$(GDB) --eval-command="target remote localhost:3333" $(PROJECT).elf

openocd_swd_erase:
	openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f1x.cfg  -c "init" -c "reset halt" -c "sleep 100" -c "stm32f1x mass_erase 0" -c "sleep 100" -c shutdown

###############################################################################

clean:
	-rm -rf $(OBJS)
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
