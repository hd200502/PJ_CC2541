/*************************************************************************
spi .h

description    : support SPIs of CC254x

created by    : hezhiwu

created date : 13-11-25

modified by   :

*************************************************************************/

#ifndef SPI_H
#define SPI_H

enum{
SPI0_ALT1 = 0,
SPI0_ALT2,
SPI1_ALT1,
SPI1_ALT2,
SPI_MAX
};

enum{
BPS_1M = 0,
BPS_2M,
BPS_3M,
BPS_4M,
BPS_MAX
};

enum{
MODE_MASTER = 0,
MODE_SLAVE,
MODE_MAX
};

enum{
FIRST_MSB = 0,
FIRST_LSB,
FIRST_MAX
};

struct spi_cfg_struct{
	uint8 * name;
	uint8 idx;
	uint8 baud;
	uint8 mode;
	uint8 msb;
};

int8 spi_register(uint8 spi_idx);
int8 spi_ctl(int8 handler, char *writebuf, uint16 wlength, char *readbuf, uint16 rlength);

#endif

