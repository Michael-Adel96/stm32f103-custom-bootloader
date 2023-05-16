/*
 * bootloader_interface.h
 *
 *  Created on: May 6, 2023
 *      Author: Dell
 */

#ifndef INC_BOOTLOADER_INTERFACE_H_
#define INC_BOOTLOADER_INTERFACE_H_

/* ----------------- Includes -----------------*/
/* ----------------- Macro Declarations -----------------*/

/* All supported commands codes */
/*==============================*/
#define CBL_GET_VER_CMD              0x10
#define CBL_GET_HELP_CMD             0x11
#define CBL_GET_CID_CMD              0x12
/* Get Read Protection Status */
#define CBL_GET_RDP_STATUS_CMD       0x13
#define CBL_GO_TO_ADDR_CMD           0x14
#define CBL_FLASH_ERASE_CMD          0x15
#define CBL_MEM_WRITE_CMD            0x16
/* Enable/Disable Write Protection */
#define CBL_ED_W_PROTECT_CMD         0x17
#define CBL_MEM_READ_CMD             0x18
/* Get Sector Read/Write Protection Status */
#define CBL_READ_SECTOR_STATUS_CMD   0x19
#define CBL_OTP_READ_CMD             0x20
/* Change Read Out Protection Level */
#define CBL_CHANGE_ROP_Level_CMD     0x21

#define CBL_VENDOR_ID                100
#define CBL_SW_MAJOR_VERSION         1
#define CBL_SW_MINOR_VERSION         1
#define CBL_SW_PATCH_VERSION         0

#define CBL_SEND_NACK                0xAB
#define CBL_SEND_ACK                 0xCD

#define CRC_TYPE_SIZE_BYTE			0x04

#define ADDRESS_IS_INVALID           0x00
#define ADDRESS_IS_VALID             0x01
/* Bootloader version */
/*==============================*/
#define CBL_VENDOR_ID                100
#define CBL_SW_MAJOR_VERSION         1
#define CBL_SW_MINOR_VERSION         1
#define CBL_SW_PATCH_VERSION         0



/**/

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

typedef void (*Jump_Ptr)(void);
/* ----------------- Software Interfaces Declarations -----------------*/
BL_Status BL_UART_Fetch_Host_Command(void);
/* ----------------- Helper Interfaces Declarations -----------------*/
void BL_Print_Message(char *format, ...);
#endif /* INC_BOOTLOADER_INTERFACE_H_ */
