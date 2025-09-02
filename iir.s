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

@ Start of executable code
.section .text

@ EE2028 Assignment 1, Sem 1, AY 2025/26
@ (c) ECE NUS, 2025

@ Write Student 1’s Name here: Mayukh Ghosh
@ Write Student 2’s Name here:

@ You could create a look-up table of registers here:

@ R0 ...
@ R1 ...

@ write your program from here:

.equ N_MAX,	10
.lcomm X_STORE, 4 * N_MAX        @ Allocate space for N_MAX x_n values
.lcomm Y_STORE, 4 * N_MAX        @ Allocate space for N_MAX y_n values

iir:
  PUSH {R4-R11, LR}

  MOV R4, R0          @ R4 = N
  MOV R5, R1          @ R5 = B[]
  MOV R6, R2          @ R6 = A[]
  MOV R7, R3          @ R7 = X_N

  LDR R8, [R5]        @ B[0]
  LDR R9, [R6]        @ A[0]
  MUL R10, R7, R8     @ R10 = X_N * B[0]
  SDIV R12, R10, R9   @ R12 = Y_N = (X_N * B[0]) / A[0]

  MOVS R0, #0         @ J = 0
LOOP_CALC:
  CMP R0, R4          @ WHILE J <= N
  BGT END_LOOP_CALC

  ADD R1, R0, #1      @ J + 1
  LSL R2, R1, #2      @ OFFSET = (J+1) * 4
  LSL R3, R0, #2      @ OFFSET = J * 4

  LDR R8, [R5, R2]    @ B[J+1]
  LDR R9, =X_STORE
  LDR R10, [R9, R3]   @ X_STORE[J]
  MUL R8, R8, R10     @ B[J+1] * X_STORE[J]

  LDR R9, [R6, R2]    @ A[J+1]
  LDR R10, =Y_STORE
  LDR R11, [R10, R3]  @ Y_STORE[J]
  MUL R9, R9, R11     @ A[J+1] * Y_STORE[J]

  SUB R8, R8, R9      @ NUMERATOR = B*X - A*Y
  LDR R9, [R6]        @ DENOMINATOR = A[0]
  SDIV R8, R8, R9     @ TERM = (B*X - A*Y) / A[0]

  ADD R12, R12, R8    @ Y_N += TERM
  ADDS R0, R0, #1     @ J++
  B LOOP_CALC

END_LOOP_CALC:
  SUBS R4, R4, #1     @ J = N - 1
SHIFT_LOOP:
  CMP R4, #0
  BLT END_SHIFT_LOOP

  LSL R0, R4, #2
  SUBS R1, R0, #4

  LDR R2, =X_STORE
  LDR R3, [R2, R1]
  STR R3, [R2, R0]    @ X_STORE[J] = X_STORE[J-1]

  LDR R2, =Y_STORE
  LDR R3, [R2, R1]
  STR R3, [R2, R0]    @ Y_STORE[J] = Y_STORE[J-1]

  SUBS R4, R4, #1
  B SHIFT_LOOP

END_SHIFT_LOOP:
  LDR R0, =X_STORE
  STR R7, [R0]        @ X_STORE[0] = X_N

  LDR R0, =Y_STORE
  STR R12, [R0]       @ Y_STORE[0] = Y_N

  MOV R0, R12
  MOV R1, #100
  SDIV R0, R0, R1     @ RETURN Y_N / 100

  POP {R4-R11, PC}

iir_base:
 	PUSH {R14}

	BL SUBROUTINE

 	POP {R14}

	BX LR

SUBROUTINE:

	BX LR
