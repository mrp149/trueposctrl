CC=gcc

#CFLAGS=-g -std=c99 -DUART -Wall -Werror
CFLAGS=-g -DUART

all:	truepos-linux

truepos-linux:	main.c truepos.c uart.c truepos.h uart.h
	$(CC) $(CFLAGS) -o $@ main.c truepos.c uart.c

clean:
	rm -f truepos-linux
