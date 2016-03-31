


void read_button_thread(void)
{
    int buttons_fd;
    int key_value;
	int ret;
	int res;
    fd_set rds;
	
	buttons_fd = open("/dev/lpc178x_btn", O_RDWR, 0);
	if(buttons_fd < 0){
		printf("open /dev/lpc178x_btn failed\n");
		return;
	}

	for(;;){
		FD_ZERO(&rds);
		FD_SET(buttons_fd, &rds);
		ret = select(buttons_fd + 1, &rds, NULL, NULL, NULL);
		if(ret < 0){
			printf("select error\n");
			return;
		}
		if(ret == 0){
			printf("select timeout\n");
			return;
		}else if(FD_ISSET(buttons_fd, &rds)){
			res = read(buttons_fd, &key_value, sizeof(key_value));
			if(res != sizeof(key_value)){
				printf("read buttons ignor\n");
				continue;
			}else{
				printf("buttons_value: 0x%x\n", key_value);
			}
		}
	}

	close(buttons_fd);
}

