#include <stdio.h>
#include <fcntl.h>

#if 0
int main(int argc, char *args[])
{
	int fd;
	int ret;

	if (argc < 3) {
		printf("usage ./mini6410_leds led_num led_of/led_off\n");
		return -1;
	}
	
	switch (args[1][0]) {
	case '0':
		fd = open("/dev/mini6410-leds0", O_RDWR);
		break;

	case '1':
		fd = open("/dev/mini6410-leds1", O_RDWR);
		break;

	case '2':
		fd = open("/dev/mini6410-leds2", O_RDWR);
		break;

	case '3':
		fd = open("/dev/mini6410-leds3", O_RDWR);
		break;
	}

	if (fd) {
		ret = ioctl(fd, args[2][0] - '0');
		if ( ret < 0)
			printf("ioctl error %d\n", ret);

		close(fd);
	}

	return 0;
}
#endif

void delay()
{
	int i = 0x500000;
	while(i--);
}

int main()
{
	int i;
	int fd0, fd1, fd2, fd3;

	fd0 = open("/dev/mini6410-leds0", O_RDWR);
	if (fd0 < 0)
		return -1;

	fd1 = open("/dev/mini6410-leds1", O_RDWR);
	if (fd1 < 0)
		return -1;

	fd2 = open("/dev/mini6410-leds2", O_RDWR);
	if (fd2 < 0)
		return -1;

	fd3 = open("/dev/mini6410-leds3", O_RDWR);
	if (fd3 < 0)
		return -1;

	ioctl(fd0, 1); //1: led off
	ioctl(fd1, 1);
	ioctl(fd2, 1);
	ioctl(fd3, 1);

	i = 10;
	while (i--) {
		ioctl(fd0, 0);
		delay();
		ioctl(fd1, 0);
		delay();
		ioctl(fd2, 0);
		delay();
		ioctl(fd3, 0);
		delay();
		ioctl(fd0, 1); //1: led off
		ioctl(fd1, 1);
		ioctl(fd2, 1);
		ioctl(fd3, 1);
		delay();
	}

	close(fd0);
	close(fd1);
	close(fd2);
	close(fd3);

	return 0;
}

