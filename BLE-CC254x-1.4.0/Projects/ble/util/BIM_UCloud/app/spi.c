/*************************************************************************
spi .c

description    : support SPIs of CC254x

created by    : hezhiwu

created date : 13-11-25

modified by   :

*************************************************************************/

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "spi.h"

//SPI0
//#define SPI0_ALT1_SUPPORT
//#define SPI0_ALT2_SUPPORT
//SPI1
//#define SPI1_ALT1_SUPPORT
#define SPI1_ALT2_SUPPORT

//#define SPI0_ALT1_CS               P0_4
//#define SPI0_ALT1_SCK             P0_5
//#define SPI0_ALT1_MISO            P0_2
//#define SPI0_ALT1_MOSI            P0_3
//#define SPI0_ALT2_CS               P1_2
//#define SPI0_ALT2_SCK             P1_3
//#define SPI0_ALT2_MISO            P1_4
//#define SPI0_ALT2_MOSI            P1_5
//#define SPI1_ALT1_CS               P0_2
//#define SPI1_ALT1_SCK             P0_3
//#define SPI1_ALT1_MISO            P0_5
//#define SPI1_ALT1_MOSI            P0_4
//#define SPI1_ALT2_CS               P1_4
//#define SPI1_ALT2_SCK             P1_5
//#define SPI1_ALT2_MISO            P1_7
//#define SPI1_ALT2_MOSI            P1_6

#if defined(SPI0_ALT1_SUPPORT) && defined(SPI0_ALT2_SUPPORT)
#error "Can not define the one when the other is defined."
#endif

#if defined(SPI1_ALT1_SUPPORT) && defined(SPI1_ALT2_SUPPORT)
#error "Can not define the one when the other is defined."
#endif


static uint8 bps_set_map[BPS_MAX][2] = {
{0x0f, 0x00}, // 1M
{0x10, 0x00}, // 2M
{0x10, 0x80}, // 3M
{0x10, 0xff} // 4M
};


static struct spi_cfg_struct spi_cfg[] = {
	#if defined(SPI0_ALT1_SUPPORT)
	{
		"spi0 alt1",
		SPI0_ALT1,
		BPS_4M,
		MODE_MASTER,
		FIRST_MSB,
	},
	#endif
	#if defined(SPI0_ALT2_SUPPORT)
	{
		"spi0 alt2",
		SPI0_ALT2,
		BPS_4M,
		MODE_MASTER,
		FIRST_MSB,
	},
	#endif
	#if defined(SPI1_ALT1_SUPPORT)
	{
		"spi1 alt1",
		SPI1_ALT1,
		BPS_4M,
		MODE_MASTER,
		FIRST_MSB,
	},
	#endif
	#if defined(SPI1_ALT2_SUPPORT)
	{
		"spi1 alt2",
		SPI1_ALT2,
		BPS_4M,
		MODE_MASTER,
		FIRST_MSB,
	},
	#endif
};

#if 1
static int8 spi_init(uint8 spi_idx)
{
	uint8 spi_num = sizeof(spi_cfg)/sizeof(struct spi_cfg_struct);
	uint8 i;

	for(i = 0; i < spi_num; i++)
	{
                if (spi_cfg[i].idx != spi_idx)
                    continue;

		switch(spi_cfg[i].idx)
		{
			case SPI0_ALT1:
				//*** Setup USART 0 SPI at alternate location 1 ***
				// USART 0 at alternate location 1
				PERCFG &= ~(0x01);
				// Peripheral function on SCK, MISO and MOSI (P0_3-5)
				P0SEL |= 0x2C;
				
				//*** Setup the SPI interface ***
				// SPI master mode
				if(spi_cfg[i].mode == MODE_MASTER)
					U0CSR = 0x00;
				else
					U0CSR = 0x20;
				// Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
				//								   data in on CPOL-inv -> CPOL
				// MSB first
				if(spi_cfg[i].msb == FIRST_MSB)
					U0GCR = 0x20;
				else
					U0GCR = 0x00;
				// SCK frequency = 3MHz (MBA250 max=10MHz, CC254x max = 4MHz)
				U0GCR |= bps_set_map[spi_cfg[i].baud][0];
				U0BAUD = bps_set_map[spi_cfg[i].baud][1];
				break;
				
			case SPI0_ALT2:
				//*** Setup USART 0 SPI at alternate location 2 ***
				// USART 0 at alternate location 2
				PERCFG |= 0x01;
				// Peripheral function on SCK, MISO and MOSI (P1_3-5)
				P1SEL |= 0x38;
				
				//*** Setup the SPI interface ***
				// SPI master mode
				if(spi_cfg[i].mode == MODE_MASTER)
					U0CSR = 0x00;
				else
					U0CSR = 0x20;
				// Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
				//								   data in on CPOL-inv -> CPOL
				// MSB first
				if(spi_cfg[i].msb == FIRST_MSB)
					U0GCR = 0x20;
				else
					U0GCR = 0x00;
				// SCK frequency = 3MHz (MBA250 max=10MHz, CC254x max = 4MHz)
				U0GCR |= bps_set_map[spi_cfg[i].baud][0];
				U0BAUD = bps_set_map[spi_cfg[i].baud][1];
				break;
				
			case SPI1_ALT1:
				//*** Setup USART 1 SPI at alternate location 1 ***
				// USART 1 at alternate location 1
				PERCFG &= ~(0x02);
				// Peripheral function on SCK, MISO and MOSI (P0_3-5)
				P0SEL |= 0x38;
				
				//*** Setup the SPI interface ***
				// SPI master mode
				if(spi_cfg[i].mode == MODE_MASTER)
					U1CSR = 0x00;
				else
					U1CSR = 0x20;
				// Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
				//								   data in on CPOL-inv -> CPOL
				// MSB first
				if(spi_cfg[i].msb == FIRST_MSB)
					U1GCR = 0x20;
				else
					U1GCR = 0x00;
				// SCK frequency = 3MHz (MBA250 max=10MHz, CC254x max = 4MHz)
				U1GCR |= bps_set_map[spi_cfg[i].baud][0];
				U1BAUD = bps_set_map[spi_cfg[i].baud][1];
				break;
				
			case SPI1_ALT2:
				//*** Setup USART 1 SPI at alternate location 1 ***
				// USART 1 at alternate location 1
				PERCFG |= 0x02;
				// Peripheral function on SCK, MISO and MOSI (P1_5-7)
				P1SEL |= 0xE0;
				
				//*** Setup the SPI interface ***
				// SPI master mode
				if(spi_cfg[i].mode == MODE_MASTER)
					U1CSR = 0x00;
				else
					U1CSR = 0x20;
				// Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
				//								   data in on CPOL-inv -> CPOL
				// MSB first
				if(spi_cfg[i].msb == FIRST_MSB)
					U1GCR = 0x20;
				else
					U1GCR = 0x00;
				// SCK frequency = 3MHz (MBA250 max=10MHz, CC254x max = 4MHz)
				U1GCR |= bps_set_map[spi_cfg[i].baud][0];
				U1BAUD = bps_set_map[spi_cfg[i].baud][1];
				break;
				
		}
	}

	return 0;
}

int8 spi_register(uint8 spi_idx)
{
	uint8 spi_num = sizeof(spi_cfg)/sizeof(struct spi_cfg_struct);
	uint8 i;

	for(i = 0; i < spi_num; i++)
	{
		if(spi_cfg[i].idx == spi_idx)
		{
                        spi_init(spi_idx);
			return i;
		}
	}

	return -1;
}

int8 spi_ctl(int8 handler, char *writebuf, uint16 wlength, char *readbuf, uint16 rlength)
{
	uint16 length;
	char * buff;

	if((handler < 0) || (handler >= (sizeof(spi_cfg)/sizeof(struct spi_cfg_struct))))
		return -1;

	switch(spi_cfg[handler].idx)
	{
		case SPI0_ALT1:
		case SPI0_ALT2:
			length = wlength;
			buff = writebuf;
			while(length--)
			{
				U0CSR &= ~0x06; 				// Clear TX_BYTE
				U0DBUF = *(buff++); 				// Write address to accelerometer
				while (!(U0CSR & 0x02));		// Wait for TX_BYTE to be set
			}

			length = rlength;
			buff = readbuf;
			while(length--)
			{
				U0CSR &= ~0x06; 				// Clear TX_BYTE
				U0DBUF = 0xff; 				// Write address to accelerometer
				while (!(U0CSR & 0x02));		// Wait for TX_BYTE to be set
				*(buff++) = U0DBUF;
			}
			break;

		case SPI1_ALT1:
		case SPI1_ALT2:
			length = wlength;
			buff = writebuf;
			while(length--)
			{
				U1CSR &= ~0x06; 				// Clear TX_BYTE
				U1DBUF = *(buff++); 				// Write address to accelerometer
				while (!(U1CSR & 0x02));		// Wait for TX_BYTE to be set
			}

			length = rlength;
			buff = readbuf;
			while(length--)
			{
				U1CSR &= ~0x06; 				// Clear TX_BYTE
				U1DBUF = 0xff; 				// Write address to accelerometer
				while (!(U1CSR & 0x02));		// Wait for TX_BYTE to be set
				*(buff++) = U1DBUF;
			}
			break;
			
		default:
			return -2;
			break;
	}

	return 0;
}

#else
#define CS              P1_4
#define SCK             P1_5
#define MISO            P1_7
#define MOSI            P1_6

int8 spi_init(void)
{
	// Peripheral function on SCK, MISO and MOSI (P1_5-7)
	P1SEL &= 0x0F;
	P1DIR |= 0x70;
	P1DIR &= 0x7F;
	SCK = 0;
	//CS = 1;
}

int8 spi_register(uint8 spi_idx)
{
	return 0;
}

#define delaytime() //{uint16 tick; for(tick = 0; tick < 1000; tick++);}
int8 spi_ctl(int8 handler, uint8 *writebuf, uint8 wlength, uint8 *readbuf, uint8 rlength)
{
	
	uint8 length;
	uint8 * buff;
	uint8 i;
	uint8 cbit = 0x80;

	SCK = 0;
	//CS = 0;
	length = wlength;
	buff = writebuf;

	while(length--)
	{
		cbit = 0x80;
		for(i = 0; i < 8; i++)
		{
			if((*buff)&cbit)
				MOSI = 1;
			else
				MOSI = 0;
			
			delaytime();

			SCK = 1;
			delaytime();
			SCK = 0;
			cbit >>= 1;
		}
		buff++;
	}

	length = rlength;
	buff = readbuf;

	while(length--)
	{
		*buff = 0;
		cbit = 0x80;
		for(i = 0; i < 8; i++)
		{
			delaytime();
			SCK = 1;
			delaytime();
			if(MISO)
				*buff |= cbit;
			SCK = 0;
			cbit >>= 1;
				
		}
		buff++;
	}

	//CS = 1;

	return 0;
}

#endif


