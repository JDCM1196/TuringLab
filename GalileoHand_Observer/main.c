/* Universidad Galileo
 * Turing Research Lab
 * Julio E. Fajardo
 * Guillermo J. Maldonado
 * Juan D. Cardona
 * Galileo Bionic Hand
 * CMSIS-DSP Application
 * Observer and Controller
 * August-21-2019
 * main.c
 */
 
 /*
 Valores importantes:
 
 R_motor = 35 ohms
 L_motor = 1.35 mH
 k_v = k_T = 3.658*10^-3 Nm/A
 t_const = 10.0 us
 J = 3.823*10^-12 Nm^2
 b = 0
 R_shunt = 3.66 ohms
 omega_ss = 2460 rad/s
 Vin = 9V
 Ip = 60 mA
 */

#define ARM_MATH_CM4

#include "MK20D7.h"                     					// Device header
#include "arm_math.h"                   					// ARM::CMSIS:DSP
#include <stdlib.h>
#include <stdio.h>
#include "drivers.h"
#include "finger.h"

fingers middle_f =  {WAITC, 3, 0, 0, 0, 62};

uint32_t ticks = 0;                                                         // 1 ms ticks
															
QuaD QD;
volatile float32_t w = 0;
float32_t V = 0;
float32_t I = 0;	
float32_t t = 0;
float32_t theta = 0;

char command[30]; 

//Inicialización de las matrices de estado
const uint8_t R = 35;
const float32_t L = 1.35f*10E-3;
const float32_t k = 3.658f*10E-3;
const float32_t J = 3.823f*10E-12;
const float32_t G = 248.98f;
const float32_t r = 0.1f;			//
const float32_t rp = 0.1f;		//
const float32_t Rs = 3.66f;

//Estados (en orden): theta, omega, corriente
float32_t A[3][3] = {0,	1,	0,
										 0,	0,	k/J,
										 0,	-k/L,	-R/L};
float32_t B[3][1] = {0,
										 0,
										 1/L};
float32_t C[1][3] = {0,	0,	1};

//Las últimas dos salidas son la fuerza y el torque ejercidos por los joints
float32_t Co[5][3] = {1/G,	0,	0,
											0,		1/G,	0,
											0,		0,	1,
											0,		0,	-k*G/rp,
											0,		0,	-k*G*r/rp};

float32_t Cc[1][3] = {0,	0,	-k*G/rp};

float32_t LG[3][1] = {-0.0177,
											0.0016,
											-0.9082};

int main(void){
  LED_Config(); 
  ADC0_Config();
  UART0_Config();
	UART1_Config();
  Output_Config();
  SysTick_Config(SystemCoreClock/100);
  PIT_Init(24);

  arm_fill_q15(0, middle_f.buffer, SIZE);

	QD_Init(&QD);
	Finger_Open(middle_f.finger_m);
	LED_On();
	
	while(1){
		/*if(w != QD.omega){
			sprintf(command, "omega: %f\n\r", QD.omega);
			UART0_putString(command);
  	}*/
	}
}
void SysTick_Handler(void) {
	V = ADC0_Read(4)*3.3f/1024.0f;
	I = V/Rs;
	theta = FTM1->CNT*PI/(G*6);
	
	sprintf(command, "%.2f,%.4f,%.2f,%.2f\r", V, I, QD.omega, theta);
	UART0_putString(command);
	
	Finger_Timing(&middle_f);
	ticks++; 
}


void PIT0_IRQHandler(){
	//char string[50];
  PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	QD_Process(&QD);
	//sprintf(string, "w: %.4f\n\r", QD.omega);
	//UART0_putString(string);
	w = QD.omega;
	//LED_Toggle();
}
