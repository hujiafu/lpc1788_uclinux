COMPILE := ../cross_tools/arm-2010q1-uclinux/bin/arm-uclinuxeabi-
CC := $(COMPILE)gcc

CFLAGS := -Wall -O2

%.o: %.c
	$(CC) $(CFLAGS) -c -o fb_test.o fb_test.c

fbtest: fb_test.o
	$(CC) $(FLAGS) -o $@ fb_test.o

clean:
	rm -rf *.o fbtest
