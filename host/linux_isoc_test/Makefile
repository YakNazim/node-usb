
CC = gcc

SOURCES=$(wildcard ./src/*.c)
BINARYS=$(SOURCES:.c=)

all: $(BINARYS) 

$(BINARYS): $(SOURCES)
	$(CC) $@.c -o $@

clean:
	rm -f $(BINARYS) 
