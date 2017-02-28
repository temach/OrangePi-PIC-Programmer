CC = gcc
CFLAGS = -g -Wall -Wextra -O0 -std=c99

all: rpp

rpp: rpp.c
	$(CC) $(CFLAGS) rpp.c -o rpp

clean:
	rm -f rpp

.PHONY: clean
