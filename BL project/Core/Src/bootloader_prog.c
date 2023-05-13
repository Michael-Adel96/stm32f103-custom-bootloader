/*
 * bootloader_prog.c
 *
 *  Created on: May 6, 2023
 *      Author: Michael Adel
 *       Brief: Bootloader Implementation for stm32f103
 */

/* ----------------- Includes -----------------*/
#include "bootloader_private.h"
#include "bootloader_interface.h"
#include "bootloader_config.h"
#include <string.h>
#include <stdarg.h>
#include "usart.h"
#include "crc.h"

/* ----------------- Static Functions Deceleration -----------------*/
static void Bootloader_Get_Version(uint8_t *Host_Buffer);
static void Bootloader_Get_Help(uint8_t *Host_Buffer);
static void Bootloader_Get_Chip_Identification_Number(uint8_t *Host_Buffer);
static void Bootloader_Read_Protection_Level(uint8_t *Host_Buffer);

static uint8_t CBL_STM32F103_Get_RDP_Level(void);
static CRC_Status Bootloader_CRC_Verify(uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC);
static void Bootloader_Send_ACK(uint8_t Replay_Len);
static void Bootloader_Send_NACK(void);
static void Bootloader_Send_Data_To_Host(uint8_t *Host_Buffer, uint32_t Data_Len);
/* ----------------- Global Variables Definitions -----------------*/
static uint8_t BL_Host_Buffer[BL_HOST_BUFFER_RX_LENGTH];

static uint8_t Bootloader_Supported_CMDs[12] = {
    CBL_GET_VER_CMD,
    CBL_GET_HELP_CMD,
    CBL_GET_CID_CMD,
    CBL_GET_RDP_STATUS_CMD,
    CBL_GO_TO_ADDR_CMD,
    CBL_FLASH_ERASE_CMD,
    CBL_MEM_WRITE_CMD,
    CBL_ED_W_PROTECT_CMD,
    CBL_MEM_READ_CMD,
    CBL_READ_SECTOR_STATUS_CMD,
    CBL_OTP_READ_CMD,
    CBL_CHANGE_ROP_Level_CMD
};
/* -----------------  Software Interfaces Definitions -----------------*/
BL_Status BL_UART_Fetch_Host_Command(void)
{
	BL_Status status = BL_NACK;
	HAL_StatusTypeDef HAL_status = HAL_ERROR;
	uint8_t data_frame_len = 0;
	/* Clear the Host Rx buffer */
	memset(BL_Host_Buffer, 0, BL_HOST_BUFFER_RX_LENGTH);

	/* Listen to the UART serial port */
	/* Expected to receive the length of the Command frame [1 Byte] */
	HAL_status = HAL_UART_Receive(BL_HOST_COMMUNICATION_UART, BL_Host_Buffer, 1, HAL_MAX_DELAY);
	if(HAL_status != HAL_OK){
		status = BL_NACK;
	} else {
		data_frame_len = BL_Host_Buffer[0];
		/* Listen to the command frame with the pre-defined length from the Host */
		HAL_status = HAL_UART_Receive(BL_HOST_COMMUNICATION_UART, &BL_Host_Buffer[1], data_frame_len, HAL_MAX_DELAY);
		if(HAL_status != HAL_OK){
			status = BL_NACK;
		} else {
			// check the received command code and execute its routine
			switch(BL_Host_Buffer[1]){
				case CBL_GET_VER_CMD:
					Bootloader_Get_Version(BL_Host_Buffer);
					status = BL_OK;
					break;
				case CBL_GET_HELP_CMD:
					Bootloader_Get_Help(BL_Host_Buffer);
					status = BL_OK;
					break;
				case CBL_GET_CID_CMD:
					Bootloader_Get_Chip_Identification_Number(BL_Host_Buffer);
					status = BL_OK;
					break;
				case CBL_GET_RDP_STATUS_CMD:
					Bootloader_Read_Protection_Level(BL_Host_Buffer);
					status = BL_OK;
					break;
				default:
					BL_Print_Message("Invalid command code received from host !! \r\n");
					break;
			}
		}
	}


}
/* ----------------- Static Functions Definitions -----------------*/
static void Bootloader_Get_Version(uint8_t *Host_Buffer)
{
	uint8_t BL_Version[4] = { CBL_VENDOR_ID, CBL_SW_MAJOR_VERSION, CBL_SW_MINOR_VERSION, CBL_SW_PATCH_VERSION };
	uint32_t host_crc32 = 0; /* the attached crc to the frame end */
	CRC_Status crc_verf_status = CRC_VERIFICATION_FAILED;
	uint16_t Host_CMD_Packet_Len = 0;

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the bootloader version from the MCU \r\n");
#endif

	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	/* CRC verification */
	host_crc32 = *((uint32_t *)(Host_Buffer + Host_CMD_Packet_Len - CRC_TYPE_SIZE_BYTES));
	crc_verf_status = Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, host_crc32);

	if(crc_verf_status == CRC_VERIFICATION_PASSED) {
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Passed \r\n");
#endif
	Bootloader_Send_ACK(4);
	Bootloader_Send_Data_To_Host((uint8_t *)(&BL_Version[0]), 4);

	} else {

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();

	}


}

static void Bootloader_Get_Help(uint8_t *Host_Buffer)
{
	uint32_t host_crc32 = 0; /* the attached crc to the frame end */
	CRC_Status crc_verf_status = CRC_VERIFICATION_FAILED;
	uint16_t Host_CMD_Packet_Len = 0;

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Get Help CMD ..... ! \r\n");
#endif

	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	/* CRC verification */
	host_crc32 = *((uint32_t *)(Host_Buffer + Host_CMD_Packet_Len - CRC_TYPE_SIZE_BYTES));
	crc_verf_status = Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, host_crc32);

	if(crc_verf_status == CRC_VERIFICATION_PASSED) {
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Passed \r\n");
#endif
	Bootloader_Send_ACK(12);
	Bootloader_Send_Data_To_Host((uint8_t *)(&Bootloader_Supported_CMDs[0]), 12);

	} else {

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();

	}
}

static void Bootloader_Get_Chip_Identification_Number(uint8_t *Host_Buffer)
{
	uint32_t host_crc32 = 0; /* the attached crc to the frame end */
	CRC_Status crc_verf_status = CRC_VERIFICATION_FAILED;
	uint16_t Host_CMD_Packet_Len = 0;
	uint16_t MCU_Identification_Number = 0;
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Get_Chip_Identification_Number..! \r\n");
#endif

	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	/* CRC verification */
	host_crc32 = *((uint32_t *)(Host_Buffer + Host_CMD_Packet_Len - CRC_TYPE_SIZE_BYTES));
	crc_verf_status = Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, host_crc32);

	if(crc_verf_status == CRC_VERIFICATION_PASSED) {
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Passed \r\n");
#endif
	/* Get the MCU chip identification number */
	MCU_Identification_Number = (uint16_t)((DBGMCU->IDCODE) & 0x00000FFF);
	Bootloader_Send_ACK(2);
	Bootloader_Send_Data_To_Host((uint8_t *)(&MCU_Identification_Number), 2);

	} else {

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("CRC Verification Failed \r\n");
#endif
	Bootloader_Send_NACK();
	}
}

static uint8_t CBL_STM32F103_Get_RDP_Level(){
	FLASH_OBProgramInitTypeDef FLASH_OBProgram;
	/* Get the Option byte configuration */
	HAL_FLASHEx_OBGetConfig(&FLASH_OBProgram);

	return (uint8_t)(FLASH_OBProgram.RDPLevel);
}

static void Bootloader_Read_Protection_Level(uint8_t *Host_Buffer){
	uint16_t Host_CMD_Packet_Len = 0;
	uint32_t Host_CRC32 = 0;
	uint8_t RDP_Level = 0;

#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
	BL_Print_Message("Read the FLASH Read Protection Out level \r\n");
#endif
	/* Extract the CRC32 and packet length sent by the HOST */
	Host_CMD_Packet_Len = Host_Buffer[0] + 1;
	Host_CRC32 = *((uint32_t *)((Host_Buffer + Host_CMD_Packet_Len) - CRC_TYPE_SIZE_BYTE));
/* CRC Verification */
	if(CRC_VERIFICATION_PASSED == Bootloader_CRC_Verify((uint8_t *)&Host_Buffer[0] , Host_CMD_Packet_Len - 4, Host_CRC32)){
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Passed \r\n");
#endif
		Bootloader_Send_ACK(1);
		/* Read Protection Level */
		RDP_Level = CBL_STM32F103_Get_RDP_Level();
		/* Report Valid Protection Level */
		Bootloader_Send_Data_To_Host((uint8_t *)&RDP_Level, 1);
	}
	else{
#if (BL_DEBUG_ENABLE == DEBUG_INFO_ENABLE)
		BL_Print_Message("CRC Verification Failed \r\n");
#endif
		Bootloader_Send_NACK();
	}
}
/* ----------------- Helper Functions Definitions -----------------*/
static CRC_Status Bootloader_CRC_Verify(uint8_t *pData, uint32_t Data_Len, uint32_t Host_CRC){
	CRC_Status crc_verf_status = CRC_VERIFICATION_FAILED;
	uint32_t MCU_CRC_Calculated = 0;
	uint8_t Data_Counter = 0;
	uint32_t Data_Buffer = 0;
	/* Calculate CRC32 */
	for(Data_Counter = 0; Data_Counter < Data_Len; Data_Counter++){
		Data_Buffer = (uint32_t)pData[Data_Counter];
		MCU_CRC_Calculated = HAL_CRC_Accumulate(CRC_ENGINE_OBJ, &Data_Buffer, 1);
	}

	/* Reset the CRC Calculation Unit */
	__HAL_CRC_DR_RESET(CRC_ENGINE_OBJ);
	/* Compare the Host CRC and Calculated CRC */
	if(MCU_CRC_Calculated == Host_CRC){
		crc_verf_status = CRC_VERIFICATION_PASSED;
	}
	else{
		crc_verf_status = CRC_VERIFICATION_FAILED;
	}

	return crc_verf_status;
}

static void Bootloader_Send_ACK(uint8_t Reply_Len){
	uint8_t Ack_Value[2] = {0};
	Ack_Value[0] = CBL_SEND_ACK;
	Ack_Value[1] = Reply_Len;
	HAL_UART_Transmit(BL_HOST_COMMUNICATION_UART, (uint8_t *)Ack_Value, 2, HAL_MAX_DELAY);
}

static void Bootloader_Send_NACK(void){
	uint8_t Ack_Value = CBL_SEND_NACK;
	HAL_UART_Transmit(BL_HOST_COMMUNICATION_UART, &Ack_Value, 1, HAL_MAX_DELAY);
}


static void Bootloader_Send_Data_To_Host(uint8_t *Host_Buffer, uint32_t Data_Len){
	HAL_UART_Transmit(BL_HOST_COMMUNICATION_UART, Host_Buffer, Data_Len, HAL_MAX_DELAY);
}

void BL_Print_Message(char *format, ...){
	char Messsage[100] = {0};
	/* holds the information needed by va_start, va_arg, va_end */
	va_list args;
	/* Enables access to the variable arguments */
	va_start(args, format);
	/* Write formatted data from variable argument list to string */
	vsprintf(Messsage, format, args);
#if (BL_DEBUG_METHOD == BL_ENABLE_UART_DEBUG_MESSAGE)
	/* Trasmit the formatted data through the defined UART */
	HAL_UART_Transmit(BL_DEBUG_UART, (uint8_t *)Messsage, sizeof(Messsage), HAL_MAX_DELAY);
#endif
	/* Performs cleanup for an ap object initialized by a call to va_start */
	va_end(args);
}
