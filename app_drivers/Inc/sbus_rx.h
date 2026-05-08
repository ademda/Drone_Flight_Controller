/*
 * sbus_rx.h
 *
 *  Low-level iBUS UART reception driver
 *  Protocol handling is done in IBUSReceiver class
 *
 *  Created on: Apr 15, 2026
 *      Author: dalya
 */

#ifndef INC_SBUS_RX_H_
#define INC_SBUS_RX_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* iBUS Protocol constants */
#define IBUS_FRAME_LENGTH			32      // Frame length in bytes
#define IBUS_FRAME_HEADER			0x20    // First byte (length)
#define IBUS_FRAME_COMMAND			0x40    // Second byte (command)
#define IBUS_MAX_CHANNELS			14      // Maximum channels in iBUS protocol


/* Low-level driver functions */
void ibus_uart_init(UART_HandleTypeDef* huart);
uint8_t* ibus_get_buffer(void);
void ibus_uart_dma_complete_callback(void);


#endif /* INC_SBUS_RX_H_ */





