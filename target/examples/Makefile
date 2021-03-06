LIBNAME	= ../usbstack
APPNAME = main

#Select target
TARGET=LPC214x
#TARGET=LPC23xx

# Tool definitions
CC      = arm-elf-gcc
LD      = arm-elf-ld -v
AR      = arm-elf-ar
AS      = arm-elf-as
CP      = arm-elf-objcopy
OD		= arm-elf-objdump
RM		= rm

# Tool flags
CFLAGS  = -I./ -I../ -c -W -Wall -Os -g -DDEBUG -D$(TARGET) -mcpu=arm7tdmi
ASFLAGS = -ahls -mapcs-32 -Wa,--defsym,$(TARGET)=1 
LFLAGS  =  -nostartfiles --warn-common
CPFLAGS = -O ihex
ODFLAGS	= -x --syms

LINKFILE	= lpc2148-rom.ld

CSRCS	= halsys.c printf.c console.c
OBJS 	= crt.o $(CSRCS:.c=.o)

EXAMPLES = hid serial msc custom serial_dave serial_dave_isoc_out isoc_dma_sample isoc_in_dma_sample isoc_ep isoc_in_dma_sample_adc isoc_in_nondma_sample isoc_out_dma_sample isoc_out_nondma_sample multiEp isoc_inout_nondma_sample_sensor_prototype

all: depend $(EXAMPLES)

hid: $(OBJS) main_hid.o $(LIBNAME).a
serial:	$(OBJS) main_serial.o serial_fifo.o armVIC.o $(LIBNAME).a
isoc_out_dma_sample: $(OBJS) isoc_out_dma_sample.o armVIC.o $(LIBNAME).a
isoc_out_nondma_sample: $(OBJS) isoc_out_nondma_sample.o armVIC.o $(LIBNAME).a
isoc_in_dma_sample: $(OBJS) isoc_in_dma_sample.o armVIC.o $(LIBNAME).a
isoc_in_dma_sample_adc: $(OBJS) isoc_in_dma_sample_adc.o adc.o armVIC.o $(LIBNAME).a
isoc_ep: $(OBJS) isoc_ep.o adc.o armVIC.o $(LIBNAME).a
isoc_inout_nondma_sample_sensor_prototype: $(OBJS) isoc_inout_nondma_sample_sensor_prototype.o adc.o armVIC.o $(LIBNAME).a
isoc_in_nondma_sample: $(OBJS) isoc_in_nondma_sample.o armVIC.o $(LIBNAME).a
isoc_dma_sample: $(OBJS) isoc_dma_sample.o armVIC.o $(LIBNAME).a
serial_dave:    $(OBJS) main_serial_dave.o serial_fifo.o armVIC.o $(LIBNAME).a
serial_dave_isoc_out:   $(OBJS) main_serial_dave_isoc_out.o serial_fifo.o armVIC.o $(LIBNAME).a

multiEp:  $(OBJS) armVIC.o  serial_fifo.o multiEp.o $(LIBNAME).a

msc:	$(OBJS) main_msc.o msc_bot.o msc_scsi.o blockdev_sd.o sdcard.o lpc2000_spi.o $(LIBNAME).a
custom:	$(OBJS) main_custom.o $(LIBNAME).a

$(EXAMPLES):
	@ echo "Building $@ example..."
	$(CC) -T $(LINKFILE) $(LFLAGS) $^ -o $@.elf -Wl,-Map,$@.map
	$(CP) $(CPFLAGS) $@.elf $@.hex
	$(OD) $(ODFLAGS) $@.elf > $@.dmp

crt.o: crt.s
	@ echo ".assembling"
	$(CC) -c $(ASFLAGS) -Wa,-ahlms=crt.lst crt.s -o crt.o

clean:
	rm -f *.hex *.elf *.o *.lst *.dmp *.map .depend

# recompile if the Makefile changes
$(OBJS): Makefile

# dependency checking
depend: $(CSRCS)
	$(CC) $(CFLAGS) -MM $^ > .depend || rm -f .depend

# phony targets
.PHONY: clean

-include .depend

