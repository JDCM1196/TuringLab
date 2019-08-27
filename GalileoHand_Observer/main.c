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

#define		SysTickFreq		100
#define		PITFreq				 12
uint32_t ticks = 0;                                                         // 1 ms ticks
															
QuaD QD;
volatile float32_t w = 0;
float32_t V = 0;
float32_t In = 0;
float32_t Ib = 0;
float32_t t = 0;
float32_t theta = 0;
float32_t omegaG = 0;

char command[50]; 

//Inicialización de las matrices de estado
const uint8_t R = 35;
const float32_t L = 1.35f*10E-3;
const float32_t k = 3.658f*10E-3;
//const float32_t J = 3.823f*10E-12;
const float32_t G = 248.98f;
const float32_t r = 4.1f*10E-3;
const float32_t rp = 4.5f*10E-3;
const float32_t Rs = 3.66f;

//Estados (en orden): theta, omega, corriente
arm_matrix_instance_f32 x;
float32_t x_f32[3][1] = {0,
													0,
													0};

arm_matrix_instance_f32 xb;
float32_t xb_f32[3][1] = {0,
													0,
													0};

arm_matrix_instance_f32 xh;
float32_t xh_f32[3][1] = {0,
													0,
													0};

arm_matrix_instance_f32 y;
float32_t y_f32[1][1] = {0};

arm_matrix_instance_f32 I;
float32_t I_f32[1][1];
//Matrices del sistema (ya discretizadas)
arm_matrix_instance_f32 A;
float32_t A_f32[3][3] = {1,	9.9993E-6,	0.3856,
												0,	0,	0,
												0,	0,	0};

arm_matrix_instance_f32 B;
float32_t B_f32[3][1] = {2.8332,
												283.6075,
												-4.3263E-19};

arm_matrix_instance_f32 C;
float32_t C_f32[1][3] = {0,	0,	1};

//Las últimas dos salidas son la fuerza y el torque ejercidos por los joints
float32_t Co[5][3] = {1/G,	0,	0,
											0,		1/G,	0,
											0,		0,	1,
											0,		0,	-k*G/rp,
											0,		0,	-k*G*r/rp};

float32_t Cc[1][3] = {0,	0,	-k*G/rp};

arm_matrix_instance_f32 LG;
float32_t LG_f32[3][1] = {-0.0296,
											-2.1085,
											-0.8462};

int main(void){
  LED_Config(); 
  ADC0_Config();
  UART0_Config();
  Output_Config();
	LED_On();
  SysTick_Config(SystemCoreClock/SysTickFreq);
  PIT_Init(PITFreq);

  arm_fill_q15(0, middle_f.buffer, SIZE);

	QD_Init(&QD);
	Finger_Open(middle_f.finger_m);
	
	while(1){
		arm_mat_init_f32(&x, 3, 1, (float32_t *)x_f32);
		arm_mat_init_f32(&xb, 3, 1, (float32_t *)xb_f32);
		arm_mat_init_f32(&xh, 3, 1, (float32_t *)xh_f32);	
		arm_mat_init_f32(&I, 1, 1, (float32_t *)I_f32);	
		
		//xb = Ax+B(11.77-I*R-L(I-Iant)/SysTickFreq)
		arm_mat_mult_f32(&A, &x, &xb);
		arm_mat_scale_f32(&B, 11.77 - In*R - L*(In-Ib)/SysTickFreq, &B);
		arm_mat_add_f32(&xb, &B, &xb);
		
		//y = Cx
		arm_mat_mult_f32(&C, &x, &y);
		
		//xh = xb-LG(y+I)
		arm_mat_add_f32(&y, &I, &y);
		arm_mat_mult_f32(&y, &LG, &xh);
		arm_mat_scale_f32(&xh, -1, &xh);
		arm_mat_add_f32(&xb, &xh, &xh);
	}
}
void SysTick_Handler(void) {
	V = ADC0_Read(4)*3.3f/1024.0f;
	In = V/Rs;
	I_f32[0][0] = In;
	theta = FTM1->CNT*PI/(G*6.0f);
	omegaG = QD.omega/G;
	
	x_f32[0][0] = theta;
	x_f32[1][0] = omegaG;
	x_f32[2][0] = In;
	
	sprintf(command, "%.2f,%.4f,%.2f,%.2f\r", V, In, omegaG, theta);
	UART0_putString(command);

	Ib = In;
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
