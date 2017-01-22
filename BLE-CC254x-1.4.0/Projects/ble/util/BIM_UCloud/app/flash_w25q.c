/*
 * flash_w25q.c
 *
 *  Created on: Jul 9, 2014
 *      Author: Administrator
 */

#include "flash_w25q.h"

#ifdef CC2541_SPI
#include <ioCC2541.h>
#include "spi.h"
#include "hal_defs.h"
#include "hal_types.h"

#else
#include "IO_Map.h"
#include "SM1.h"
#include "io.h"

extern void *pSM1;
extern volatile bool SM1_FinishBlockTX;
extern volatile bool SM1_FinishBlockRX;
#endif

#define W25Q80    0XEF13   
#define W25Q16    0XEF14
#define W25Q32    0XEF15
#define W25Q64    0XEF16

uint16_t SPI_FLASH_TYPE;    //定义我们使用的 flash 芯片型号   

#define DummyData          0xff

/* Private typedef -----------------------------------------------------------*/
//#define SPI_FLASH_PageSize      4096
#define SPI_FLASH_PageSize      		256
#define SPI_FLASH_PerWritePageSize      256

/* Private define ------------------------------------------------------------*/
#define W25X_WriteEnable		      0x06 
#define W25X_WriteDisable		      0x04 
#define W25X_ReadStatusReg		      0x05 
#define W25X_WriteStatusReg		      0x01 
#define W25X_ReadData			      0x03 
#define W25X_FastReadData		      0x0B 
#define W25X_FastReadDual		      0x3B 
#define W25X_PageProgram		      0x02 
#define W25X_BlockErase			      0xD8 
#define W25X_SectorErase		      0x20 
#define W25X_ChipErase			      0xC7 
#define W25X_PowerDown			      0xB9 
#define W25X_ReleasePowerDown	   	  0xAB 
#define W25X_DeviceID			      0xAB 
#define W25X_ManufactDeviceID   	  0x90 
#define W25X_JedecDeviceID		      0x9F 



#ifdef CC2541_SPI
//#define CS              P1_4
#define CS              P0_4
#define SCK             P1_5
#define MISO            P1_7
#define MOSI            P1_6

#define CS_DISABLED     1
#define CS_ENABLED      0

static uint8 acc_initialized = FALSE;
static int8 spi_handler = -1;

#define Flash_CS_Low() {CS = CS_ENABLED;P1_4 = 1;}
#define Flash_CS_High() {CS = CS_DISABLED;}

#else
void inline Flash_CS_Low()
{
	GPIOD_PDOR &=~GPIO_PDOR_PDO(1<<4);
}

void inline Flash_CS_High()
{
	GPIOD_PDOR |= GPIO_PDOR_PDO(1<<4);
}
#endif

void SPI_FLASH_Init(void)
{
	if(acc_initialized)
		return;

	acc_initialized = 1;

    // Configure CS (P0_4) as output
	P0SEL &= ~(0x10);
    P0DIR |= 0x10;
	
	P1SEL &= ~(0x10);
	P1DIR |=  (0x10); // P1_4 output
	P1_4 = 1;
	
	spi_handler = spi_register(SPI1_ALT2);

	Flash_CS_High();
}


#define WIP_Flag                  0x01  /* Write In Progress (WIP) flag */
#define SET                       0x01
#define RESET					  0x00
#define Dummy_Byte                0xFF

/*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
  //dprintf("%s %x\n", __FUNCTION__, SectorAddr);
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();
  SPI_FLASH_WaitForWriteEnd();
  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();
  /* Send Sector Erase instruction */
  SPI_FLASH_SendByte(W25X_SectorErase);
  /* Send SectorAddr high nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();
  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BulkErase(void)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();

  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();
  /* Send Bulk Erase instruction  */
  SPI_FLASH_SendByte(W25X_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}
void SPI_FLASH_BulkErase_ex(void)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();

  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();
  /* Send Bulk Erase instruction  */
  SPI_FLASH_SendByte(W25X_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();

  /* Wait the end of Flash writing */
  //SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_PageWrite(uint8* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Enable the write access to the FLASH */
  SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();
  /* Send "Write to Memory " instruction */
  SPI_FLASH_SendByte(W25X_PageProgram);
  /* Send WriteAddr high nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  if(NumByteToWrite > SPI_FLASH_PerWritePageSize)
  {
     NumByteToWrite = SPI_FLASH_PerWritePageSize;
     //printf("\n\r Err: SPI_FLASH_PageWrite too large!");
  }

#ifdef CC2541_SPI
	spi_ctl(spi_handler, pBuffer, NumByteToWrite, NULL, 0);
#else
  /* while there is data to be written on the FLASH */
  while(!SM1_FinishBlockTX);
  SM1_FinishBlockTX=FALSE;
  SM1_SendBlock(pSM1,pBuffer,NumByteToWrite);
  
  while(!SM1_FinishBlockTX);
#endif  
  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferWrite(uint8* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
      {
        temp = NumOfSingle - count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
      }
      else
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferRead(uint8* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);

  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);

#ifdef CC2541_SPI
	spi_ctl(spi_handler, NULL, 0, pBuffer, NumByteToRead);
	//spi_ctl(spi_handler, pBuffer, NumByteToRead, NULL, 0);
#else
  while(!SM1_FinishBlockTX);
  SM1_FinishBlockTX=FALSE;
  SM1_FinishBlockRX=FALSE;
  SM1_ReceiveBlock(pSM1,pBuffer, NumByteToRead); 
  SM1_SendBlock(pSM1,pBuffer,NumByteToRead);
  while(!SM1_FinishBlockRX);
#endif
  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
uint32_t SPI_FLASH_ReadID(void)
{
  uint8_t  Temp[3];
  uint32_t TempID= 0;

  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(W25X_JedecDeviceID);

  Temp[0]=SPI_FLASH_ReadByte();
  Temp[1]=SPI_FLASH_ReadByte();
  Temp[2]=SPI_FLASH_ReadByte();
  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();

  #if 1
  for(uint8 i = 0; i < 3; i++)
  {
    TempID <<= 8;
    TempID |= Temp[i];
  }
  #else
  TempID = (Temp[0]<< 16) | (Temp[1] << 8) | Temp[2];
  #endif

  return TempID;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
uint32_t SPI_FLASH_ReadDeviceID(void)
{
  uint32_t Temp = 0;

  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(W25X_DeviceID);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  
  /* Read a byte from the FLASH */
  Temp = SPI_FLASH_ReadByte();

  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();

  return Temp;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_StartReadSequence
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr)
{
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);

  /* Send the 24-bit address of the address to read from -----------------------*/
  /* Send ReadAddr high nibble address byte */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
uint8 SPI_FLASH_ReadByte(void)
{
  uint8 result=0;

#ifdef CC2541_SPI
	spi_ctl(spi_handler, NULL, 0, &result, 1);
#else
  SM1_FinishBlockRX=FALSE;
  SM1_ReceiveBlock(pSM1,&result, 1); 
  SPI_PDD_WriteData8Bit(SPI1_BASE_PTR, DummyData);
  while(!SM1_FinishBlockRX);
#endif

  return result;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
uint8 SPI_FLASH_SendByte(uint8 byte)
{
#ifdef CC2541_SPI
	spi_ctl(spi_handler, &byte, 1, NULL, 0);
#else
  /* Loop while DR register in not emplty */

  /* Send byte through the SPI1 peripheral */
  while(!SM1_FinishBlockTX);
  SM1_FinishBlockTX=FALSE;
  SM1_SendBlock(pSM1,&byte,1);
  while(!SM1_FinishBlockTX);
  /* Return the byte read from the SPI bus */
#endif
  return 0;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendHalfWord
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
uint16_t SPI_FLASH_SendHalfWord(uint16_t HalfWord)
{
#ifdef CC2541_SPI
	uint8 data[2];

	data[0] = HalfWord>>8;
	data[1] = HalfWord&0xff;
	spi_ctl(spi_handler, data, 2, NULL, 0);
#else
  /* Loop while DR register in not emplty */
  uint8_t dataH=(uint8_t)(HalfWord>>8);
  uint8_t dataL=(uint8_t)(HalfWord&0xff);
  
  while(!SM1_FinishBlockTX);
  SM1_FinishBlockTX=FALSE;
  SM1_SendBlock(pSM1,&dataH,1);
  
  while(!SM1_FinishBlockTX);
  SM1_FinishBlockTX=FALSE;
  SM1_SendBlock(pSM1,&dataL,1);

  /* Wait to receive a Half Word */
  
  /* Return the Half Word read from the SPI bus */
#endif
  return 0;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(void)
{
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "Write Enable" instruction */
  SPI_FLASH_SendByte(W25X_WriteEnable);
  

  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteDisable(void)
{
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "Write Enable" instruction */
  SPI_FLASH_SendByte(W25X_WriteDisable);
  

  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WaitForWriteEnd(void)
{
  uint8 FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "Read Status Register" instruction */
  SPI_FLASH_SendByte(W25X_ReadStatusReg);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI_FLASH_ReadByte();

  }
  while ((FLASH_Status & WIP_Flag) == SET); /* Write in progress */

  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();
}


//进入掉电模式
void SPI_Flash_PowerDown(void)   
{ 
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "Power Down" instruction */
  SPI_FLASH_SendByte(W25X_PowerDown);

  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();
}   

//唤醒
void SPI_Flash_WAKEUP(void)   
{
  /* Select the FLASH: Chip Select low */
  Flash_CS_Low();

  /* Send "Power Down" instruction */
  SPI_FLASH_SendByte(W25X_ReleasePowerDown);

  /* Deselect the FLASH: Chip Select high */
  Flash_CS_High();                   //等待TRES1
}   

#if 0

typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* 获取缓冲区的长度 */
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)
#define countof(a)      (sizeof(a) / sizeof(*(a)))
#define  BufferSize (countof(Tx_Buffer)-1)

#define  FLASH_WriteAddress     0x00000
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress
#define  sFLASH_ID              0xEF4014// 0xEF4015

/* 发送缓冲区初始化 */
uint8_t Tx_Buffer[] = "华邦串行flash 检测\r\n";
uint8_t Rx_Buffer[BufferSize];

uint32_t DeviceID = 0;
uint32_t FlashID = 0;
TestStatus TransferStatus1 = FAILED;

// 函数原型声明
void Delay(uint32_t nCount)
{
	uint16_t cnt;
	
	while(nCount--)
	{
		cnt = 0;
		while(cnt--);
	}
}
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

static uint8 buff_tmp[128];
#define SECTOR_SIZE ((uint32)4096)
uint32 addr;
uint16 sn;
static uint8 ttt = 0;
void SPI_Flash_test2(void)
{
#if 1
	for(sn = 0; sn < 256; sn++)
	{
		addr = sn * SECTOR_SIZE;

		//SPI_FLASH_SectorErase(addr);
		SPI_FLASH_BufferRead((uint8 *)buff_tmp, addr, 128);
		ttt++;

	}

#endif
}
void SPI_Flash_test(void)
{
	#if 1
	uint16 i;

	SPI_FLASH_Init();
	#if 1
	for(sn = 0; sn < 256; sn++)
	{
		addr = sn * SECTOR_SIZE;

		SPI_FLASH_SectorErase(addr);
		//SPI_FLASH_BufferRead((uint8 *)buff_tmp, addr, 32);
		ttt++;

	}
	#endif
	#else
	/* 2M串行flash W25X16初始化 */
	  SPI_FLASH_Init();
	  
	  /* Get SPI Flash Device ID */
	  DeviceID = SPI_FLASH_ReadDeviceID();
	  
	  Delay( 5 );

	  /* Get SPI Flash ID */
	  FlashID = SPI_FLASH_ReadID();

	  printf("\r\n FlashID is 0x%X,  Manufacturer Device ID is 0x%X\r\n", FlashID, DeviceID);

	  /* Check the SPI Flash ID */
	  if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF3015 */
	  {
	  
	    printf("\r\n 检测到华邦串行flash W25X16 !\r\n");
	    
	    /* Erase SPI FLASH Sector to write on */
	    SPI_FLASH_SectorErase(FLASH_SectorToErase);	 	
	    
	    SPI_FLASH_BulkErase();
	   
	    /* 将发送缓冲区的数据写到flash中 */
	    SPI_FLASH_BufferWrite(Tx_Buffer, FLASH_WriteAddress, BufferSize);
			printf("\r\n 写入的数据为：%s \r\t", Tx_Buffer);

	    /* 将刚刚写入的数据读出来放到接收缓冲区中 */
	    SPI_FLASH_BufferRead(Rx_Buffer, FLASH_ReadAddress, BufferSize);
			printf("\r\n 读出的数据为：%s \r\n", Rx_Buffer);

	    /* 检查写入的数据与读出的数据是否相等 */
	    TransferStatus1 = Buffercmp(Tx_Buffer, Rx_Buffer, BufferSize);

			if( PASSED == TransferStatus1 )
	    {    
	        printf("\r\n 2M串行flash(W25X16)测试成功!\n\r");
	    }
	    else
	    {        
	        printf("\r\n 2M串行flash(W25X16)测试失败!\n\r");
	    }
	  }// if (FlashID == sFLASH_ID)
	  else
	  {    
	    printf("\r\n 获取不到 W25X16 ID!\n\r");
	  }

	  SPI_Flash_PowerDown();  
	  #endif
}


/*
 * 函数名：Buffercmp
 * 描述  ：比较两个缓冲区中的数据是否相等
 * 输入  ：-pBuffer1     src缓冲区指针
 *         -pBuffer2     dst缓冲区指针
 *         -BufferLength 缓冲区长度
 * 输出  ：无
 * 返回  ：-PASSED pBuffer1 等于   pBuffer2
 *         -FAILED pBuffer1 不同于 pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return PASSED;
}
#endif

