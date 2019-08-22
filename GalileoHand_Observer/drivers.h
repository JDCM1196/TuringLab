/* Universidad Galileo
 * Turing Research Lab
 * Julio E. Fajardo
 * Galileo Bionic Hand
 * CMSIS-DSP Application
 * Embedded Prostheses Controller
 * May-09-2017
 * drivers.h
 */
 
#ifndef DRIVERS_H_
#define DRIVERS_H_


#define ARM_MATH_CM4
#include <stdint.h>
#include <stdlib.h>
#include "arm_math.h"                   // ARM::CMSIS:DSP

void LED_Config(void);
void Output_Config(void);
void Switch_Config(void);
void ADC0_Config(void);
void FTM0_Config(void);
void UART0_Config(void);
void UART1_Config(void);

void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

void PIT_Init(uint32_t frequency);

typedef struct Quad{
	uint16_t a1;
	uint16_t a2;
	float32_t omega; 
} QuaD;

void QD_Init(QuaD *QD);
void QD_Process(QuaD *QD);

uint16_t ADC0_Read(unsigned int channel);
void UART0_send(char dato);
void UART0_putString(char *mystring);
void UART1_send(char dato);
void UART1_putString(char *mystring);

#endif /* DRIVERS_H_ */
