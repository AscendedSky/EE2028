/******************************************************************************
 * @project        : EE2028 Assignment 1 Program Template
 * @file           : main.c
 * @original author: CK Tham, ECE NUS
 * @modified by    : Ni Qingqing, ECE NUS
 * @brief          : Main program body
 *
 * @description    : This code is based on work originally done by CK Tham.
 *                   Modifications were made to update functionality and
 *                   adapt to current course requirements.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "stdio.h"

#define N_MAX 10
#define X_SIZE 12
// Enable CYCCNT register
#define start_timer()  *((volatile int*)0xE0001000) = 0x40000001

// Disable CYCCNT register
#define stop_timer()   *((volatile int*)0xE0001000) = 0x40000000

// Get value from CYCCNT register
#define get_timer()    *((volatile int*)0xE0001004)

// Necessary function to enable printf() using semihosting
extern void initialise_monitor_handles(void);

extern int iir(int N, int* b, int* a, int x_n); // asm implementation
int iir_c(int N, int* b, int* a, int x_n); // reference C implementation

int main(void)
{
	// Necessary function to enable printf() using semihosting
	initialise_monitor_handles();

	//variables
	int i;
	int N = 9;

	int it1, it2, it3, it4;
	int ans1, ans2;

	//  NOTE: DO NOT modify the code below this line
	// think of the values below as numbers of the form y.yy (floating point with 2 digits precision)
	// which are scaled up to allow them to be used integers
	// within the iir function, we divide y by 100 (decimal) to scale it down
//	int b[N_MAX+1] = {100, 250, 360, 450, 580}; //N+1 dimensional feedforward
//	int a[N_MAX+1] = {100, 120, 180, 230, 250}; //N+1 dimensional feedback
//	int x[X_SIZE] = {100, 230, 280, 410, 540, 600, 480, 390, 250, 160, 100, 340};

//	int b[N_MAX+1] = {100, 250, 360, 450, 580};
//	int a[N_MAX+1] = {100, 120, 180, 230, 250};
//	int x[X_SIZE] = {100};

//	int b[N_MAX+1] = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 5};
//	int a[N_MAX+1] = {100, 95, 85, 75, 65, 55, 45, 35, 25, 15, 5};
//	int x[X_SIZE] = {50, 100, 150, 200, 250, 300, -300, -200, -100, 0, 100, 200};

//	int b[N_MAX+1] = {100, 250, 360, 450, 580};
//	int a[N_MAX+1] = {100, 120, 180, 230, 250};
//	int x[X_SIZE] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

	int b[N_MAX+1] = {100, 250, 360, 450, 580};
	int a[N_MAX+1] = {100, 120, 180, 230, 250};
	int x[X_SIZE] = {100, -100, 200, -200, 300, -300, 400, -400, 0, 0, 100, -100};

	// Call assembly language function iir for each element of x
	for (i=0; i<X_SIZE; i++)
	{
		start_timer();
		it1 = get_timer();
		ans1 = iir(N, b, a, x[i]);
		it2 = get_timer() - it1;  // Derive the cycle count difference
		it3 = get_timer();
		ans2 = iir_c(N, b, a, x[i]);
		it4 = get_timer() - it3;  // Derive the cycle count difference
		stop_timer();

		printf("asm Cycles taken: %d\n", it2);
		printf( "asm: i = %d, y_n = %d, \n", i, ans1) ;
		printf("C Cycles taken: %d\n", it4);
		printf( "C  : i = %d, y_n = %d, \n", i, ans2) ;
	}
	while (1); //halt
}

int iir_c(int N, int* b, int* a, int x_n)
{ 	// The implementation below is inefficient and meant only for verifying your results.

	static int x_store[N_MAX] = {0}; // to store the previous N values of x_n.
	static int y_store[N_MAX] = {0}; // to store the previous values of y_n.

	int j;
	int y_n;

	y_n = x_n*b[0]/a[0];

	for (j=0; j<N; j++)
	{
		y_n+=(b[j+1]*x_store[j]-a[j+1]*y_store[j])/a[0];
	}

	for (j=N-1; j>0; j--)
	{
		x_store[j] = x_store[j-1];
		y_store[j] = y_store[j-1];
	}

	x_store[0] = x_n;
	y_store[0] = y_n;

	y_n /= 100; // scaling down

	return y_n;
}
