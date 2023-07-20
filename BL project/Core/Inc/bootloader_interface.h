/*
 * bootloader_interface.h
 * 
 *  Created on: May 6, 2023
 *      Author: Michael Adel
 *       Brief: Bootloader Implementation for stm32f103
 */


/*
 *  
 */
#ifndef INC_BOOTLOADER_INTERFACE_H_
#define INC_BOOTLOADER_INTERFACE_H_

/* ----------------- Includes -----------------*/
/* ----------------- Macro Declarations -----------------*/

/* All supported commands codes */
#define CBL_GET_VER_CMD              0x10
#define CBL_GET_HELP_CMD             0x11
#define CBL_GO_TO_ADDR_CMD           0x14
#define CBL_FLASH_ERASE_CMD          0x15
#define CBL_MEM_WRITE_CMD            0x16
#define CBL_JUMP_USER_APP_1			 0x22
#define CBL_JUMP_USER_APP_2			 0x23
#define CBL_MEM_WRITE_APP_1_CMD      0x24
#define CBL_MEM_WRITE_APP_2_CMD      0x25
#define CBL_FLASH_ERASE_APP1_CMD     0x26
#define CBL_FLASH_ERASE_APP2_CMD     0x27

/* ACK codes */
#define CBL_SEND_NACK                0xAB
#define CBL_SEND_ACK                 0xCD

#define CRC_TYPE_SIZE_BYTE			 0x04

/* Address checkers constants */
#define ADDRESS_IS_INVALID           0x00
#define ADDRESS_IS_VALID             0x01

/* MEM Access constants */
#define INVALID_SECTOR_NUMBER        0x00
#define VALID_SECTOR_NUMBER          0x01
#define UNSUCCESSFUL_ERASE           0x02
#define SUCCESSFUL_ERASE             0x03

/* CBL_MEM_WRITE_CMD */
#define FLASH_PAYLOAD_WRITE_FAILED   0x00
#define FLASH_PAYLOAD_WRITE_PASSED   0x01

/* ----------------- Macro Functions Declarations -----------------*/

/* ----------------- Data Type Declarations -----------------*/
typedef enum{
	BL_NACK = 0,
	BL_OK
}BL_Status;

typedef enum{
	CRC_VERIFICATION_FAILED = 0,
	CRC_VERIFICATION_PASSED
}CRC_Status;

typedef enum{
	APP_ID_1 = 0,
	APP_ID_2
}APP_ID;

typedef void (*Jump_Ptr)(void);
typedef void (*pMainApp)(void);
/* ----------------- Software Interfaces Declarations -----------------*/
void BL_Init(void);
BL_Status BL_UART_Fetch_Host_Command(void);

/* ----------------- Helper Interfaces Declarations -----------------*/
void BL_Print_Message(char *format, ...);

#endif /* INC_BOOTLOADER_INTERFACE_H_ */
