/*
 * sendbuffer.s
 *
 *  Created on: Nov 26, 2025
 *      Author: Amin
 */

.syntax unified

.global sendBuffer

.section .text.rodata

/**
*
* void sendBuffer(UART_HandleTypeDef *huart, const uint8_t *buffer, uint16_t size)
*
* R0 = pointer to USART registers
* R1 = pointer to buffer
* R2 = size
*/

sendBuffer:
	PUSH {R4-R6}
	MOV R6, #0

loop:
	CMP R6, R2
	BGE end
	LDRB R4, [R1, R6]

not_empty:
	LDR R5, [R0, #0x1C]
	TST R5, #(1 << 7)
	BEQ not_empty
	STR R4, [R0, #0x28]
	ADD R6, R6, #1
	B loop

not_tc:
	LDR R5, [R0, #0x1C]
	TST R5, #(1 << 6)
	BNE not_tc

end:
	POP {R4-R6}
	BX LR
