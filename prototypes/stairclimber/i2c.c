#include "i2c.h"

int i2c_init(const char *device_path) {
	int fd;
	
	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		perror("Failed to open port");
		exit(1);
	}
	
	return fd;
}

void i2c_write(int fd, int addr, uint8_t *c, int length) {

	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		perror("Failed to connect to slave");
		exit(1);
	}

	if (write(fd, c, 1) != 1) {
		perror("Failed to write");
		exit(1);
	}
}

void i2c_read(int fd, int slave_addr, int reg_addr, int length, uint8_t *buf) {

	unsigned int i;
	for (i = 0; i < length; i++) {
		char c[1] = {reg_addr + i};
		i2c_write(fd, slave_addr, c, 1);
		if (read(fd, c, 1) != 1) {
			perror("Failed to read");
			exit(1);
		}
		buf[i] = c[0];
	}
}

