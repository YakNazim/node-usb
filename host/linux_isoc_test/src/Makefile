
CC		= gcc
LD              = $(CC)

AR		= ar rvs

RM              = rm -f

# CFLAGS	= -I./include -c -Wall -fwhole-program -combine -O2 -g
CFLAGS          =  -I./include -c -Wall -O0 -g

SRCS		= $(wildcard ./*.c)

OBJS            = $(SRCS:.c=.o)

PROGS           = $(SRCS:.c=) 

.PHONY: clean

.SUFFIXES : .o  

.c.o :
	@echo "...compiling" $<
	$(CC) $(CPPFLAGS) -c $<

all:  $(PROGS)

$(PROGS): $(OBJS)
	@echo "...linking" $@ 
	$(LD) -o $@ $@.o

clean:
	$(RM)  $(PROGS) $(OBJS) a.out


