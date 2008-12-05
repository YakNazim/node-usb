

CC       = gcc
LD       = $(CC)

INCLUDE  = ./include
CFLAGS   = -Wall -I$(INCLUDE)

#SOURCES=$(wildcard ./src/*.c)

MULTI   = lpc_util.c \
          multiEp.c

MULTIOBJS=$(MULTI:.c=.o)

ALLOBJS = $(MULTIOBJS) 

PROGS   = multiEp

all:  $(PROGS)

.c.o :
#	@echo "...compiling" $<
	$(CC) $(CFLAGS) -c $<


multiEp: $(MULTIOBJS)
	$(LD) -o $@ $(MULTIOBJS)

clean:
	rm -f $(ALLOBJS) $(PROGS)  *~ ./include/*~
