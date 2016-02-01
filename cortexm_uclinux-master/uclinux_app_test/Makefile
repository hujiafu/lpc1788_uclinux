COMPILE := ../cross_tools/arm-2010q1-uclinux/bin/arm-uclinuxeabi-
CC := $(COMPILE)gcc

EXEC = fbtest
OBJS = fb_test.o
CFLAGS += -I ../filesystem/rootfs/root/usr/include
CFLAGS += -mcpu=cortex-m3 -mthumb
LIBS := -L ../filesystem/rootfs/root/usr/lib
LDFLAGS += -elf2flt

%.o: %.c
	$(CC) -o $(CFLAGS) -c -o fb_test.o fb_test.c

all:
	$(CC) -o fbtest fb_test.c $(CFLAGS) $(LIBS)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o
