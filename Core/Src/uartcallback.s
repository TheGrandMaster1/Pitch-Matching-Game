/*
 * uartcallback.s
 *
 *  Created on: Nov 27, 2025
 *      Author: Admin
 */


.syntax unified
.global HAL_UART_RxCpltCallback
.type HAL_UART_RxCpltCallback, %function
.section .text

HAL_UART_RxCpltCallback:
    PUSH {R4-R7, LR}

    LDR R1, [R0]
    LDR R2, =0x40013800
    CMP R1, R2
    BNE end_callback
    LDR R3, =uart_rx_buffer
    LDRB R4, [R3]
    MOV R5, #13
    CMP R4, R5
    BEQ newline_case
    MOV R5, #10
    CMP R4, R5
    BEQ newline_case
    LDR R6, =uart_pos
    LDR R7, [R6]
    LDR R5, =uart_line
    MOV R1, #64
    SUB R1, R1, #1
    CMP R7, R1
    CMP R7, R1
    BGE skip_store
    ADD R5, R5, R7
    STRB R4, [R5]
    ADD R7, R7, #1
    STR R7, [R6]

skip_store:
	B restart_rx

newline_case:
	LDR R6, =uart_pos
	LDR R7, [R6]
	LDR R5, =uart_line
	ADD R5, R5, R7
	MOV R1, #0
	STRB R1, [R5]
	LDR R5, =uart_cmd_ready
	MOV R1, #1
	STR R1, [R5]
	MOV R1, #0
	STR R1, [R6]

restart_rx:
	LDR R0, =huart1
	LDR R1, =uart_rx_buffer
	MOV R2, #1
	BL HAL_UART_Receive_IT

end_callback:
	POP {R4-R7, LR}
	BX LR
