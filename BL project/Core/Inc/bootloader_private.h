/*
 * bootloader_private.h
 *
 *  Created on: May 7, 2023
 *      Author: Dell
 */

#ifndef INC_BOOTLOADER_PRIVATE_H_
#define INC_BOOTLOADER_PRIVATE_H_

#define DATA_FRAME_LENGTH_MAX 255

/* CRC_VERIFICATION */
/*=============================*/
#define CRC_ENGINE_OBJ               &hcrc
#define CRC_TYPE_SIZE_BYTES          4


/* Bootloader version */
/*==============================*/
#define CBL_VENDOR_ID                100
#define CBL_SW_MAJOR_VERSION         1
#define CBL_SW_MINOR_VERSION         1
#define CBL_SW_PATCH_VERSION         0

typedef enum {
	MEM_WRITE_FAILED = 0,
	MEM_WRITE_DONE
} MEM_WRITE_STATUS;

typedef enum {
	STATUS_NOK = 0,
	STATUS_OK
} BL_STATUS;

typedef enum {
	BL_only = 0,
	BL_APP1,
	BL_APP2,
	BL_APP1_APP2,
	BL_ERR_STATUS
} BL_APPS_STATUS;
#endif /* INC_BOOTLOADER_PRIVATE_H_ */
