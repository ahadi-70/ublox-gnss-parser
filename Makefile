CC      = gcc
CFLAGS  = -Wall -Wextra -Wpedantic -std=c11 -Iinclude -g -O2
LDFLAGS = -lm

SRCS    = src/gnss_parser.c src/nmea_parser.c src/ubx_parser.c
TEST    = test/test_gnss_parser.c

.PHONY: all test clean

all: test

test: $(SRCS) $(TEST)
	$(CC) $(CFLAGS) -o test_gnss $(SRCS) $(TEST) $(LDFLAGS)
	./test_gnss

clean:
	rm -f test_gnss
