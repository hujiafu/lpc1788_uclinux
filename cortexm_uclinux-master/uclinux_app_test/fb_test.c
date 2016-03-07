//#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/io.h>
//#include <linux/fb.h>
//#include <sys/mman.h>

int main()
{

	int ret, fd, i, j;
	char read_data[32];
	char write_data[32];
	char offset;
	//struct fb_var_screeninfo vinfo;
	//struct fb_fix_screeninfo finfo;
	printf("Hello world\n");

	fd = open("/sys/devices/platform/lpc2k-i2c.0/i2c-0/0-0050/eeprom", O_RDWR);
	if(fd < 0){
		printf("Open at24c02 fail\n");
		return -1;
	}

	for(i=0; i<32; i++)
		write_data[i] = i + 1;

	lseek(fd, 0, SEEK_SET);

	ret = write(fd, write_data, 32);
	if(ret < 0){
		printf("write error\n");
	}
	
	lseek(fd, 0, SEEK_SET);

	ret = read(fd, read_data, 32);
	if(ret < 0){
		printf("read error\n");
		return -1;
	}else if(ret < 32){
		printf("%d\n", ret);
		return -1;
	}

	for(i=0; i<32; i++){
		if(i%6 == 0)
		  printf("\n");
		printf(" %03d", read_data[i]);

	}
	printf("\n");
	return 0;



#if 0
	fp = open("/dev/fb0", O_RDWR);

	if(fp < 0){
		printf("Error: Can not open framebuffer device\n");
		exit(1);
	}

	if(ioctl(fp, FBIOGET_FSCREENINFO, &finfo)){
		printf("Error reading fixed information\n");
		exit(2);
	}

	if(ioctl(fp, FBIOGET_VSCREENINFO, &vinfo)){
		printf("Error reading variable information\n");
		exit(3);
	}

	printf("The mem is :%d\n", finfo.smem_len);
	printf("The line_length is :%d\n", finfo.line_length);
	printf("The xres is :%d\n", vinfo.xres);
	printf("The yres is :%d\n", vinfo.yres);
	printf("bits_per_pixel is :%d\n", vinfo.bits_per_pixel);
	close(fp);
#endif
}

