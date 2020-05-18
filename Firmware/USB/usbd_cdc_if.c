
#include "usbd_cdc_if.h"

#define APP_RX_DATA_SIZE  512
#define APP_TX_DATA_SIZE  512

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
	CDC_Init_FS,
	CDC_DeInit_FS,
	CDC_Control_FS,
	CDC_Receive_FS
};

static int8_t CDC_Init_FS(void)
{
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
	return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
	return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	switch(cmd)
	{
		case CDC_SEND_ENCAPSULATED_COMMAND:
			break;
		case CDC_GET_ENCAPSULATED_RESPONSE:
			break;
		case CDC_SET_COMM_FEATURE:
			break;
		case CDC_GET_COMM_FEATURE:
			break;
		case CDC_CLEAR_COMM_FEATURE:
			break;

		/*******************************************************************************/
		/* Line Coding Structure                                                       */
		/*-----------------------------------------------------------------------------*/
		/* Offset | Field       | Size | Value  | Description                          */
		/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
		/* 4      | bCharFormat |   1  | Number | Stop bits                            */
		/*                                        0 - 1 Stop bit                       */
		/*                                        1 - 1.5 Stop bits                    */
		/*                                        2 - 2 Stop bits                      */
		/* 5      | bParityType |  1   | Number | Parity                               */
		/*                                        0 - None                             */
		/*                                        1 - Odd                              */
		/*                                        2 - Even                             */
		/*                                        3 - Mark                             */
		/*                                        4 - Space                            */
		/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
		/*******************************************************************************/

		case CDC_SET_LINE_CODING:
			break;
		case CDC_GET_LINE_CODING:
			break;
		case CDC_SET_CONTROL_LINE_STATE:
			break;
		case CDC_SEND_BREAK:
			break;
		default:
			break;
	}

	return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
	uint8_t result = USBD_OK;

	if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
		return USBD_FAIL;

	if (((Buf == NULL) || (Len == NULL)) || (*Len <= 0))
		return USBD_FAIL;

	do result = USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	while (result != USBD_OK);

	do result = USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	while (result != USBD_OK);

	CDC_Receive_CB(Buf, *Len);

	//USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	//USBD_CDC_ReceivePacket(&hUsbDeviceFS);

	return (USBD_OK);
}

uint8_t CDC_Transmit_FS_Ready(void)
{
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
	return (hcdc == NULL) ? USBD_BUSY : (hcdc->TxState != 0) ? USBD_BUSY : USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
	uint8_t result = USBD_OK;

	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
	//if (hcdc == NULL)
	//	return USBD_BUSY;
	if (hcdc->TxState != 0)
	  return USBD_BUSY;

	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

	return result;
}
