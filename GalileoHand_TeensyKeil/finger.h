/* Universidad Galileo
 * Turing Research Lab
 * Julio E. Fajardo
 * Galileo Bionic Hand
 * CMSIS-DSP Application
 * Embedded Prostheses Controller
 * May-09-2017
 * finger.h
 */
 
#ifndef FINGER_H_
#define FINGER_H_

#include <stdint.h>
#include <stdlib.h>
#include "arm_math.h"                   // ARM::CMSIS:DSP

#define 		SIZE     50				//50ms sample size

#define			OPEN      0				//cuando se está abriendo
#define			WAITC     1				//está abierto, esperando a cerrarse
#define			CLOSE     2				//se está cerrando
#define			WAITO     3				//está cerrado

#define			POWER     0
#define			HOOK      1
#define			PINCH     2
#define			POINT     3
#define			LATERAL   4
#define			PEACE     5
#define 		ROCK      6
#define		  TWO			  7
#define			THREE			8
#define			FOUR			9
#define			FANCY			10
#define			INDEX			11
#define			REST			12

typedef struct finger{
	uint8_t state;
	uint8_t finger_m;		//qué dedo es, según el número del motor que lo controla
	uint32_t time_ms;		//tiempo que se tarda en realizar una acción
	uint32_t time_r;		//cambio de tiempo que sobra -> diferencia de tiempo de OPEN y CLOSE
	q15_t mean;					//mean absolute value con punto fijo para la corriente
	q15_t threshold;		//threshold que depende de cada dedo
	q15_t buffer[SIZE];
	} fingers;

void Finger_Close(uint8_t finger_m);
void Finger_Open(uint8_t finger_m);
void Finger_Stop(uint8_t finger_m);
void Finger_Action(fingers * finger_f, uint8_t action);
void Finger_Timing(fingers * finger_f);
void Finger_Rotation(fingers *finger_f, uint8_t action);
void Hand_Action(uint8_t hand_action);
void Finger_ActionTime(fingers *finger_f, uint8_t action);

#endif /* DRIVERS_H_ */
