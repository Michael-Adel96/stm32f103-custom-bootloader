/*
 * bootloader_config.h
 *
 *  Created on: May 6, 2023
 *      Author: Dell
 */

#ifndef INC_BOOTLOADER_CONFIG_H_
#define INC_BOOTLOADER_CONFIG_H_


/* Flash configurations */
/* Set the start address of the user app */
#define FLASH_SECTOR_APP_1_BASE_ADDRESS   0x8005000U
#define FLASH_SECTOR_APP_2_BASE_ADDRESS   0x800A800U
#define FLASH_LAST_PAGE_NUM				63


/* Debug Configurations */
#define BL_DISABLE_UART_DEBUG_MESSAGE   0x00
#define BL_ENABLE_UART_DEBUG_MESSAGE    0x01
#define BL_DEBUG_METHOD  BL_ENABLE_UART_DEBUG_MESSAGE
#define BL_DEBUG_UART                &huart2

/* Host Communication configuration */
#define BL_HOST_COMMUNICATION_UART   &huart3

/* Host receiver buffer length */
#define BL_HOST_BUFFER_RX_LENGTH     200

/* Send debug info on serial port */
#define DEBUG_INFO_DISABLE           0
#define DEBUG_INFO_ENABLE            1
#define BL_DEBUG_ENABLE              DEBUG_INFO_ENABLE


#endif /* INC_BOOTLOADER_CONFIG_H_ */
