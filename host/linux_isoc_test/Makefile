
CC = gcc

SOURCES=$(wildcard ./src/*.c)
BINARYS=$(SOURCES:.c=)

all: $(BINARYS)

$(BINARYS):
	$(CC) $@.c -o $@

clean:
	rm -f $(BINARYS) 
