#ifndef BBB_H
#define BBB_H

#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#ifdef __cplusplus
extern "C" {
#endif

int  i2c_init(const char *device_path);
void i2c_write(int fd, int addr, uint8_t *c, int length);
void i2c_read(int fd, int slave_addr, int reg_addr, int length, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif

