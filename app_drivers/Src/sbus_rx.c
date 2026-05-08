/*
 * sbus_rx.c
 *
 *  Low-level iBUS UART reception driver
 *  Handles DMA-based reception of iBUS frames
 *  Protocol validation is done in IBUSReceiver class
 *
 *  Created on: Apr 15, 2026
 *      Author: dalya
 */

#include "sbus_rx.h"
#include "Receiver.h"
#include <string.h>

/* Static buffer for DMA reception */
//uint8_t ibus_dma_buffer[IBUS_FRAME_LENGTH] = {0};
//UART_HandleTypeDef* p_uart_handle = NULL;
//extern RC_Command_t rc_cmd;
//void ibus_uart_init(UART_HandleTypeDef* huart)
//{
//	if (huart == NULL) {
//		return;
//	}
//
//	p_uart_handle = huart;
//
//	/* Start DMA reception on UART */
//	HAL_UART_Receive_DMA(p_uart_handle, ibus_dma_buffer, IBUS_FRAME_LENGTH);
//}
//
//
//void ibus_uart_dma_complete_callback(void)
//{
//	if (p_uart_handle != NULL) {
//		/* Restart DMA for continuous reception */
//		//parse_received_data(&(rc_cmd.ibus_data));
//		get_commands(&rc_cmd);
//		HAL_UART_Receive_DMA(p_uart_handle, ibus_dma_buffer, IBUS_FRAME_LENGTH);
//	}
//}
//
//
