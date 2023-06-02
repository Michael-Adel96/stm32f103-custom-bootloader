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

#endif /* INC_BOOTLOADER_PRIVATE_H_ */
