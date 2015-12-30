# AndriXSWC Program
# Implements AXCP and AXDP protocols and manages the UART interface
# Author: Christoph Krofitsch

CC = gcc
CFLAGS = -Wall -Wextra -g -std=c99 -pedantic -D_BSD_SOURCE -D_POSIX_SOURCE

PROGRAM = andrixswc
OBJ = tools.o axcp.o ringbuffer.o andrixswc.o
SRC = $(OBJ:%.o=%.c)

all: $(PROGRAM) userprogram.o andrixhwtype1.o andrixhwtype2.o andrixhwtype3.o

$(PROGRAM) : $(OBJ)
	$(CC) -o $@ $^

$.o: $.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -fR $(OBJ) userprogram.o andrixhwtype1.o andrixhwtype2.o andrixhwtype3.o $(PROGRAM)
