
/*
 * iir.s
 *
 *  Created on: 29/7/2025
 *      Author: Ni Qingqing
 */

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global iir

.section .text

@ EE2028 Assignment 1, Sem 1, AY 2025/26
@ (c) ECE NUS, 2025

@ Student 1: Mayukh Ghosh
@ Student 2: Arnav Mahajan

@ -----------------------------
@ Current Register Usage:
@ R0  - Return value (y_n / 100)
@ R1  - TEMP / constant
@ R2  - Offset (J * 4)
@ R3  - TEMP
@ R4  - N (Filter order)
@ R5  - Pointer to B[]
@ R6  - Pointer to A[]
@ R7  - x_n (current input)
@ R8  - TEMP
@ R9  - Base address of X_STORE
@ R10 - Base address of Y_STORE
@ R11 - Offset for (J+1) * 4
@ R12 - y_n (accumulated output)
@ -----------------------------

@ Proposed Register Usage:
@ R0  - N (Filter order) Change to return value at end
@ R1  - Pointer to B[]
@ R2  - Pointer to A[]
@ R3  - x_n (current input)
@ R4  - TEMP for B[i] x_n
@ R5  - TEMP for A[i] y_n
@ R6  - Base address of X_STORE
@ R7 - Base address of Y_STORE
@ R8 - y_n (accumulated output)
@ R9 - Offset i*4
@ R10  - Offset (head - i)*4
@ R11  - TEMP
@ R12  - TEMP


.equ N_MAX, 10
.align 5               @ Align on 32-byte boundary for cache efficiency
.lcomm X_STORE, 4 * (N_MAX)
.align 5               @ Align on 32-byte boundary for cache efficiency
.lcomm Y_STORE, 4 * (N_MAX) @Need to update this back to N_MAX by figuring out how to remove the oldest value

.data
.align 2
head: .word 0

.text


iir:
  PUSH {R4-R12, LR}
  CMP R0, #0
  BEQ STORE_CURRENT     @ If N=0, return y_n
  LDR R6, =X_STORE  @ Base of X_STORE
  LDR R7, =Y_STORE @ Base of Y_STORE
  MOV R8, #0
  MOV R9, #1
  LDR R11, =head
  LDR R10, [R11]
  MOV R4, N_MAX*4
  SDIV R5, R10, R4 @head div nmax*4
  CMP R5, #0
  BGT BUFFER_FULL
  LSR R10, R10, #2
  SUB R10, R10, R9

LOOP_UNFULL:

	CMP R9, R0
    BGT STORE_CURRENT  @ for (i = 1; i < N; i++)

    CMP R10, #0
    BLT STORE_CURRENT

	LSL R10, R10, #2
	LSL R9, R9, #2

    LDR R4, [R1, R9]        @ B[i]
    LDR R11,  [R6, R10]        @ X_STORE[n-i]
    LDR R5,  [R2, R9]        @ A[i]
    LDR R12, [R7, R10]        @ Y_STORE[n-i]
    MUL R4, R4, R11          @ B[i] * x
    MLS R5, R5, R12, R4     @ (B*x - A*y)
    ADD R8, R8, R5 @ Accumulate y_n
    LSR R10, R10, #2
    LSR R9, R9, #2
    ADD R9, R9, #1
    SUB R10, R10, #1
    B LOOP_UNFULL

BUFFER_FULL:
	@4 % 3 = 1
	MLS R10, R5, R4, R10
	LSR R10, R10, #2
	SUB R10, R10, R9 @4 % 3 = 1


PRE_LOOP_FULL:
	CMP R9, R0
    BGT STORE_CURRENT  @ for (i = 1; i <= N; i++)

    ADD R10, R10, #N_MAX
    CMP R10, #N_MAX
    BLT LOOP_FULL
    SUB R10, R10, #N_MAX

LOOP_FULL:

	LSL R10, R10, #2
	LSL R9, R9, #2
    LDR R4, [R1, R9]        @ B[i]
    LDR R11,  [R6, R10]        @ X_STORE[n-i]
    LDR R5,  [R2, R9]        @ A[i]
    LDR R12, [R7, R10]        @ Y_STORE[n-i]
    MUL R4, R4, R11          @ B[i] * x
    MLS R5, R5, R12, R4     @ (B*x - A*y)
    ADD R8, R8, R5 @ Accumulate y_n
    LSR R10, R10, #2
    LSR R9, R9, #2
    ADD R9, R9, #1
    SUB R10, R10, #1
    B PRE_LOOP_FULL

STORE_CURRENT:

	LDR R4, [R1]
	LDR R5, [R2]
	MLA R8, R3, R4, R8
	SDIV R8, R8, R5
	LDR R11, =head
	LDR R12, [R11]
	MOV R4, N_MAX*4
    SDIV R9, R12, R4 @head div nmax*4
    MLS R10, R9, R4, R12 @head - quotient*nmax*4
	STR R3, [R6, R10]     @ X_STORE[head] = x_n
    STR R8, [R7, R10]     @ Y_STORE[head] = y_n
    MOV R0, R8
    MOV R1, #100
    SDIV R0, R0, R1
    ADD R12, R12, #4
    STR R12, [R11]
	POP {R4-R12, PC}
