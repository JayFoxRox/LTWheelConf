OBJS=main.o wheelfunctions.o wheels.o
LIBS=-lusb

CC=clang

CFLAGS=-m32 -Wall -g -O0


all: ltwheelconf

ltwheelconf: $(OBJS)
	$(CC) $(CFLAGS) $(LIBS) -o ltwheelconf $(OBJS)

main.o: main.c
	$(CC) $(CFLAGS) -c main.c

wheels.o: wheels.c wheels.h
	$(CC) $(CFLAGS) -c wheels.c


wheelfunctions.o: wheelfunctions.c wheelfunctions.h wheels.h
	$(CC) $(CFLAGS) -c wheelfunctions.c

clean:
	rm -rf ltwheelconf $(OBJS)
