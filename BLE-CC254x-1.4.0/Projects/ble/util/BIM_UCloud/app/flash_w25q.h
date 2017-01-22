/*
 * flash_w25q.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Administrator
 */

#ifndef FLASH_W25Q_H_
#define FLASH_W25Q_H_

#define CC2541_SPI

#ifdef CC2541_SPI
#include "hal_types.h"
#define uint8_t uint8
#define uint16_t uint16
#define uint32_t uint32
#else
#include "PE_Types.h"
#endif

#define FLASH_FIRST_BOLCK_START_ADDRESSS   0x000000
#define FLASH_SECOND_BOLCK_START_ADDRESSS  0x010000
#define FLASH_THIRD_BOLCK_START_ADDRESSS   0x020000
#define FLASH_FOUR_BOLCK_START_ADDRESSS    0x030000
#define FLASH_FIVE_BOLCK_START_ADDRESSS    0x040000
#define FLASH_SIX_BOLCK_START_ADDRESSS     0x050000
#define FLASH_SEVEN_BOLCK_START_ADDRESSS   0x060000
#define FLASH_EIGHT_BOLCK_START_ADDRESSS   0x070000

#define FLASH_END_ADDRESS      0x07ff00

#define SETOR_SIZE                  	   0x001000

#define available_Start_Address  0
//#define HZK_Start_Address        0
#define HZK_font_Start_address 	 45120
#define Num6X10_Start_Address    261696
#define Num7X12_Start_Address    261796
#define Num8X16_Start_Address    261916
#define Num16X27_Start_Address   262076
#define AsiicChar_Start_Address  262616


#define gImage_battery_1  		 264664
#define gImage_blueteeth 		 265112
#define gImage_exercise_icon 	 265264
#define gImage_heart_rate_icon   270840
#define gImage_noenough_battery  276416
#define gImage_set_call_icon     276544
#define gImage_set_date_icon     281304
#define gImage_set_factory_model 286064
#define gImage_set_icon          290824
#define gImage_set_language_icon 296400
#define gImage_set_message_icon  301160
#define gImage_set_save_icon     305920
#define gImage_sleep_icon        310680
#define gImage_message 		     316256
#define gImage_phone 			 329992

#define gImage_clear 			 343728
#define gImage_Email 			 348488
#define gImage_little_voice      362224
#define gImage_so                366600
#define gImage_start             375008

#define gImage_1 				 379768
#define gImage_2 				 381288
#define gImage_3 				 382808
#define gImage_card 			 384328
#define gImage_run               389904  //392172



#define gImage_main_voice_icon    392180
#define gImage_eye 				  397756
#define gImage_eye_icon           401124
#define gImage_main_find_icon     405884
#define gImage_Phone              411460
#define gImage_Main_music         415020
#define gImage_Last               420596
#define gImage_Next               421404
#define gImage_Start              422212
#define gImage_Stop               426972
#define gImage_schedule_main_icon 431732

#define NUM_xSIZE_6				  6
#define NUM_xSIZE_7				  7
#define NUM_xSIZE_8				  8
#define NUM_xSIZE_16			  16

#define MAXSIZE_NUM               0
#define MEDIUMSIZE_NUM			  1
#define NORMALSIZE_NUM			  2


#define MAXSIZE_NUM_ADDRESS       196272
#define MEDIUMSIZE_NUM_ADDRESS	  196812
#define ASSIC_ADDRESS			  197212

#ifdef AsiicChar_Start_Address
#undef AsiicChar_Start_Address
#define AsiicChar_Start_Address   ASSIC_ADDRESS
#endif



#define Pedometer_Icon 		198748
#define Music_Icon 			198863
#define Stopwatch_Icon 		198978
#define Card_Icon 			199093
#define Schedule_Icon 		199208
#define Setting_Icon 		199323
#define HeartRate_Icon 		199438
#define Voice_Icon          199553
#define FindPhone_Icon      199668

#define card_sig_bmp                199783
#define card_case_bmp 				199843
#define clear_bmp 					199947
#define play_bmp 					200122
#define pause_bmp 					200297
#define last_bmp 					200472
#define next_bmp 					200500
#define audio_bmp 					200528
#define	short_msg_notify_bmp 		200600
#define call_notify_bmp 			200704
#define real_message_bmp 			200808
#define email_notify_bmp 			200912
#define	schedule_notify_bmp 		201016
#define anti_lost_notify_bmp 		201120
#define language_setting_bmp 		201224
#define save_mode_bmp 				201328
#define factory_mode_bmp 			201432


#define Card_Info_Address		   (FLASH_EIGHT_BOLCK_START_ADDRESSS+0x00f000+2048)  //最后2KB      我的名片信息
#define CardCase_Info_Address	   (FLASH_EIGHT_BOLCK_START_ADDRESSS+0x00f000) 	    //最后  Sector      256Blok 存放名片夹 信息

typedef struct
{
	uint32 current_write_address;        //可用数据的结束地址
	uint32 current_read_address;		 //读取的当前地址
	uint32 current_read_address_end;     //读取的结束地址
	uint32 numPedometer;				 //总共可用数据
	uint32 CurrentSetor;				 //当前扇区
	uint32 number_of_WriteCycles;		 //写循环次数
	uint32 number_of_ReadCycles;		 //读循环次数
}Pedometer_Info;

#define Pedometer_MAX_Num_Per_Page  21

#define Pedometer_Start_Address     (FLASH_EIGHT_BOLCK_START_ADDRESSS+0x004000)    //计步 在flash中的起始地址
#define Pedometer_End_Address 	    (FLASH_EIGHT_BOLCK_START_ADDRESSS+0x00E000)  //计步 在 flash中的结束地址  小于这个地址    //共 40KB
#define Pedometer_Info_Address      (FLASH_EIGHT_BOLCK_START_ADDRESSS+0x00E000)    //计步 在flash中的信息                               



#define Schedule_Start_Address      (FLASH_EIGHT_BOLCK_START_ADDRESSS+0x001000)    //语音日程存放的信息
#define Schedule_Bak_Start_Address  (FLASH_EIGHT_BOLCK_START_ADDRESSS+0x002000)    //语音日程备份的地址。。
#define Schedule_info_Address		(FLASH_EIGHT_BOLCK_START_ADDRESSS+0x003000)    //信息记录的地址

typedef struct 
{
	uint8    current_writesecion; // 当前写的段
	uint32   HZK_startaddress;// 
	uint32   Num6X10_startaddress;
	uint32   Num7x12_startaddress;
	uint32   Num8x16_startaddress;
	uint32   Num16x27_startaddress;
	uint32   AsiicChar_startaddress;
}Flash_Info;



void SPI_FLASH_Init(void);
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_BulkErase_ex(void);
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t SPI_FLASH_ReadID(void);
uint32_t SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);


uint8_t SPI_FLASH_ReadByte(void);
uint8_t SPI_FLASH_SendByte(uint8_t byte);
uint16_t SPI_FLASH_SendHalfWord(uint16_t HalfWord);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WriteDisable(void);
void SPI_FLASH_WaitForWriteEnd(void);


void SPI_Flash_test(void);
	
#endif /* FLASH_W25Q_H_ */
