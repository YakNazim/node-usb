
CC = gcc

SOURCES=$(wildcard ./src/*.c)
BINARYS=$(SOURCES:.c=)

all: $(BINARYS) 
	make -C multiEp

$(BINARYS):
	$(CC) -g $@.c -o $@

clean:
	rm -f $(BINARYS) 
