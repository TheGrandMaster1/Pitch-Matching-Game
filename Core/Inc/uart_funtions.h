/*
 * uart_funtions.h
 *
 *  Created on: Nov 26, 2025
 *      Author: Admin
 */

#ifndef INC_UART_FUNTIONS_H_
#define INC_UART_FUNTIONS_H_

extern void sendBuffer(USART_TypeDef *USARTx, const uint8_t *buffer, uint16_t size);

extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);



#endif /* INC_UART_FUNTIONS_H_ */
