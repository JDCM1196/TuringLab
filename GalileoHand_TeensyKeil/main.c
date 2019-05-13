/* Universidad Galileo
 * Turing Research Lab
 * Julio E. Fajardo
 * Galileo Bionic Hand
 * CMSIS-DSP Application
 * Embedded Prostheses Controller
 * May-09-2017
 * main.c
 */

#define ARM_MATH_CM4

#include "MK20D7.h"                     					// Device header
#include "arm_math.h"                   					// ARM::CMSIS:DSP
#include <stdlib.h>
#include <stdio.h>
#include "drivers.h"
#include "finger.h"

#define DEACTIVATED 0
#define ACTIVATED   1

int16_t value = 0;
char debug[32];

int16_t encoding = 0;
															//cosas para el demo
volatile char receivedCMD;
char command[10]; 
char insind;
char activate=0;
															///cosas para el demo
typedef struct electrode{
  q15_t mean;
  q15_t threshold;
  q15_t buffer[SIZE];
  q15_t rectified[SIZE];
  } electrodes;

electrodes E1 = {0,100};
electrodes E2 = {0,50};

uint8_t muscle_state = DEACTIVATED;

fingers thumb_rot = {WAITC,6,0,200,0,80};
fingers thumb_f =   {WAITC,5,0,200,0,80};
fingers index_f =   {WAITC,4,0,200,0,80};
fingers middle_f=  {WAITC,3,0,200,0,100};
fingers ring_f  =   {WAITC,2,0,200,0,80};
fingers little_f=   {WAITC,1,0,200,0,80};

const uint8_t actions[8][6] = { CLOSE, CLOSE, CLOSE, CLOSE, CLOSE, CLOSE,   // Power Grip 
                                CLOSE, CLOSE, CLOSE, OPEN,  CLOSE, CLOSE,   // Point
                                OPEN,  OPEN,  OPEN,  CLOSE, CLOSE, CLOSE,   // Pinch
                                CLOSE, CLOSE, CLOSE, CLOSE, OPEN,  OPEN,    // Hook
                                CLOSE, CLOSE, CLOSE, CLOSE, CLOSE, OPEN,    // Lateral
                                CLOSE, CLOSE, OPEN,  OPEN,  CLOSE, CLOSE,   // Peace
                                OPEN,  OPEN,  OPEN,  OPEN,  OPEN,  OPEN,	  // Rest
																OPEN,  OPEN,  OPEN,  CLOSE, OPEN,  OPEN			// ~(Point)
                              };

uint8_t btn = 0;                                                            // Activate / deactivate 
uint8_t cmd = 0;                                                            // LCD commands
uint32_t ticks = 0;                                                         // 1 ms ticks
uint8_t i = 0;

int count = 0;

int main(void){
  Switch_Config();
  LED_Config(); 
  ADC0_Config();
  UART0_Config();
	UART1_Config();
  Output_Config();
  SysTick_Config(SystemCoreClock/1000);
  PIT_Init(1000);

  arm_fill_q15(0, little_f.buffer, SIZE);
  arm_fill_q15(0, ring_f.buffer, SIZE);
  arm_fill_q15(0, middle_f.buffer, SIZE);
  arm_fill_q15(0, index_f.buffer, SIZE);
  arm_fill_q15(0, thumb_f.buffer, SIZE);
	
  //arm_fill_q15(0, E1.buffer, SIZE);
  //arm_fill_q15(0, E2.buffer, SIZE);
	
  while(1){
		LED_Toggle();
		
	  if(receivedCMD){	//si se recibió información para la pantalla
			//Código para cambiar acción con serial
			if(command[0]=='n'){
				//LED_On();
				if(cmd<5) cmd++;
				else cmd = 0;
				UART0_send(cmd+'0');
			}else if(command[0]=='p'){
				//LED_Off();
				if(cmd==0) cmd=5;
				else cmd--;
				UART0_send(cmd+'0');
			}else if(command[0]=='a'){
        //if(E1.mean>E1.threshold){
				//LED_On();
				activate=1;
      }else if(command[0]=='d'){
        //if(E2.mean>E2.threshold){
				//LED_Off();
				activate = 0;
				Hand_Action(REST);
      }else if(command[0] == 'b'){

			}else{
				Hand_Action(REST);
			}
			
			if(activate){
				LED_Off();
				switch(cmd){
					case POWER:    Hand_Action(POWER);    break;
					case POINT:    Hand_Action(POINT);    break;		
					case PINCH:    Hand_Action(PINCH);    break;
					case HOOK:     Hand_Action(HOOK);     break;
					case LATERAL:  Hand_Action(LATERAL);  break;
					case PEACE:    Hand_Action(PEACE);    break;
					case NOTPOINT: Hand_Action(NOTPOINT); break;
					default:       Hand_Action(REST); 		LED_On();
				}
			}
			receivedCMD=0;//myo
			UART0_send('\r');
			UART0_send('\n');
			command[0] = 0;
		}
	}
    //Finger_Close(1);
    //Finger_Open(1);
}
void SysTick_Handler(void) {
  //LED_On();		
	little_f.buffer[ticks%SIZE] = (int16_t) ADC0_Read(2);
	ring_f.buffer[ticks%SIZE]   = (int16_t) ADC0_Read(3);
	middle_f.buffer[ticks%SIZE] = (int16_t) ADC0_Read(4);
	index_f.buffer[ticks%SIZE]  = (int16_t) ADC0_Read(5);
	thumb_f.buffer[ticks%SIZE]  = (int16_t) ADC0_Read(6);

	Finger_Timing(&little_f);
	Finger_Timing(&ring_f);
	Finger_Timing(&middle_f);
	Finger_Timing(&index_f);
	Finger_Timing(&thumb_rot);
	
	ticks++; 
  //LED_Off();
}

//Aquí hay que cambiarlo para adaptarlo para bluetooth
void PIT0_IRQHandler(void){
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;																										
	E1.buffer[ticks%SIZE] = ADC0_Read(0)-2850;
	E2.buffer[ticks%SIZE] = ADC0_Read(1)-2850;
  //sprintf(debug,"%d\r",ADC0_Read(0)-2048);
	//UART0_putString(debug);
  //LED_Toggle();
}

void PORTC_IRQHandler(void){
	if(PORTC->PCR[6]&PORT_PCR_ISF_MASK){
		if(cmd<5) cmd++;
		else cmd = 0;
		UART0_send(cmd+'0');
		PORTC->PCR[6] |= (PORT_PCR_ISF_MASK);
	}
	if(PORTC->PCR[7]&PORT_PCR_ISF_MASK){
		if(btn<1) {
			btn++;
			LED_On();
		}
		else {
			btn = 0;
			LED_Off();
	}
		PORTC->PCR[7] |= (PORT_PCR_ISF_MASK);
	}
}
	

void UART1_RX_TX_IRQHandler(void){
	uint8_t data;
	(void) UART1->S1;
	data = UART1->D;
	if(data=='\n' || data=='\r'){
		receivedCMD = 1;
	} else{
		command[0]=data;
	}
}

void Hand_Action(uint8_t hand_action){
	Finger_Action(&little_f, actions[hand_action][0]);
	Finger_Action(&ring_f, actions[hand_action][1]);
	Finger_Action(&middle_f, actions[hand_action][2]);
	Finger_Action(&index_f, actions[hand_action][3]);
	Finger_Action(&thumb_f, actions[hand_action][4]);
	Finger_ActionTime(&thumb_rot, actions[hand_action][5]);
}
