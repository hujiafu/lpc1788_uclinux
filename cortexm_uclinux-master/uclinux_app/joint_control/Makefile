COMPILE := arm-none-linux-gnueabi-
CC := $(COMPILE)gcc
CFLAGS := -Wall -O2
CFLAGS += -I/home/hujiafu/openssl/openssl-1.0.1p/include
CFLAGS += -I/usr/local/json-c-arm/include/json-c
CFLAGS += -I/home/hujiafu/file_system/console
LIBS := /usr/local/ssl/lib/libssl.a \
/usr/local/ssl/lib/libcrypto.a \
/usr/local/json-c-arm/lib/libjson-c.a

%.o: %.c
	$(CC) $(CFLAGS) -c -o websocket.o websocket.c
	#$(CC) $(CFLAGS) -c -o websocket.o nopoll-regression-client.c
	$(CC) $(CFLAGS) -c -o nopoll.o nopoll.c
	$(CC) $(CFLAGS) -c -o nopoll_conn.o nopoll_conn.c
	$(CC) $(CFLAGS) -c -o nopoll_ctx.o nopoll_ctx.c
	$(CC) $(CFLAGS) -c -o nopoll_decl.o nopoll_decl.c
	$(CC) $(CFLAGS) -c -o nopoll_io.o nopoll_io.c
	$(CC) $(CFLAGS) -c -o nopoll_listener.o nopoll_listener.c
	$(CC) $(CFLAGS) -c -o nopoll_log.o nopoll_log.c
	$(CC) $(CFLAGS) -c -o nopoll_loop.o nopoll_loop.c
	$(CC) $(CFLAGS) -c -o nopoll_msg.o nopoll_msg.c
	$(CC) $(CFLAGS) -c -o ssl_test.o ssl_test.c
	$(CC) $(CFLAGS) -c -o yunattInterface.o yunattInterface.c
	$(CC) $(CFLAGS) -c -o camera_test.o camera_test.c
	$(CC) $(CFLAGS) -c -o capture1.o capture1.c


websocket: websocket.o $(LIBS)
	$(CC) $(FLAGS) -o $@ websocket.o nopoll.o nopoll_conn.o nopoll_ctx.o \
		nopoll_decl.o nopoll_io.o nopoll_listener.o \
		nopoll_log.o nopoll_loop.o nopoll_msg.o yunattInterface.o -L. $(LIBS)

	$(CC) $(FLAGS) -o ssltest ssl_test.o -L. $(LIBS)
	
	$(CC) $(FLAGS) -o cameratest camera_test.o -L. $(LIBS)
	
	$(CC) $(FLAGS) -o capture capture1.o -L. $(LIBS)

clean:
	rm -rf *.o websocket ssltest cameratest capture
