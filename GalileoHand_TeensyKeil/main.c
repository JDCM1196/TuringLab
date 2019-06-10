/* Universidad Galileo
 * Turing Research Lab
 * Julio E. Fajardo
 * Guillermo J. Maldonado
 * Juan D. Cardona
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
															
volatile char receivedCMD;
char command[30]; 
char activate = 0;

uint8_t muscle_state = DEACTIVATED;

fingers thumb_rot = {WAITC,6,0,500,0,65};
fingers thumb_f =   {WAITC,5,0,0,0,65};
fingers index_f =   {WAITC,4,0,0,0,70};
fingers middle_f =  {WAITC,3,0,0,0,62};
fingers ring_f  =   {WAITC,2,0,0,0,73};
fingers little_f=   {WAITC,1,0,0,0,65};

const uint8_t actions[9][6] = { CLOSE, CLOSE, CLOSE, CLOSE, CLOSE, CLOSE,   // Power Grip 
                                CLOSE, CLOSE, CLOSE, OPEN,  CLOSE, CLOSE,   // Point
                                OPEN,  OPEN,  OPEN,  CLOSE, CLOSE, CLOSE,   // Pinch
                                CLOSE, CLOSE, CLOSE, CLOSE, OPEN,  OPEN,    // Hook
                                CLOSE, CLOSE, CLOSE, CLOSE, CLOSE, OPEN,    // Lateral
                                CLOSE, CLOSE, OPEN,  OPEN,  CLOSE, CLOSE,   // Peace
                                OPEN,  OPEN,  OPEN,  OPEN,  OPEN,  OPEN,	  // Rest
																OPEN,  OPEN,  OPEN,  CLOSE, OPEN,  OPEN,		// ~(Point)
																OPEN,	 CLOSE,	CLOSE, OPEN,	CLOSE, CLOSE		// Rock
                              };

uint8_t cmd = 0;                                                            // LCD commands
uint32_t ticks = 0;                                                         // 1 ms ticks
int ready = 0;
float flt;

int main(void){
  LED_Config(); 
  ADC0_Config();
  UART0_Config();
	UART1_Config();
  Output_Config();
  SysTick_Config(SystemCoreClock/1000);

  arm_fill_q15(0, little_f.buffer, SIZE);
  arm_fill_q15(0, ring_f.buffer, SIZE);
  arm_fill_q15(0, middle_f.buffer, SIZE);
  arm_fill_q15(0, index_f.buffer, SIZE);
  arm_fill_q15(0, thumb_f.buffer, SIZE);
	
  //arm_fill_q15(0, E1.buffer, SIZE);
  //arm_fill_q15(0, E2.buffer, SIZE);

  while(1){	
		
		//if(ready){
			if(activate){
				LED_On();
				switch(cmd){
					case POWER:    Hand_Action(POWER);    break;
					case POINT:    Hand_Action(POINT);    break;		
					case PINCH:    Hand_Action(PINCH);    break;
					case HOOK:     Hand_Action(HOOK);     break;
					case LATERAL:  Hand_Action(LATERAL);  break;
					case PEACE:    Hand_Action(PEACE);    break;
					//case ROCK:		 Hand_Action(ROCK);			break;
					default:       Hand_Action(REST); 		LED_On();
				}
				
					arm_q15_to_float(&(index_f.mean),&flt,1);
					sprintf(command, "mean:%f %d\n\r",flt,index_f.mean);
					UART0_putString(command);
					sprintf(command, "time_ms: %d\n\r", index_f.time_ms);
					UART0_putString(command);
					sprintf(command, "time_r: %d\n\r", index_f.time_r);
					UART0_putString(command);
				
			} else{
				LED_Off();
				Hand_Action(REST);
			}
		//}
	}
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
	Finger_Timing(&thumb_f);
	Finger_Timing(&thumb_rot);
	ticks++; 
  //LED_Off();
}


void UART1_RX_TX_IRQHandler(void){
	uint8_t data;
	(void) UART1->S1;
	data = UART1->D;
	//LED_Toggle();
	
	if(data == '\n' || data == '\r'){
		receivedCMD = 1;
	} else{
		switch(data){
			case 'a':{
				activate = ACTIVATED; 
				break;
			}
			case 'd':{
				activate = DEACTIVATED; 
				break;
			}
			case 'n':{
				if(!activate){
					if(cmd < 5) cmd++;
					else cmd = 0;
					UART0_send(cmd + '0');
				}
				break;
			}
			case 'p':{
				if(!activate){
					if(cmd > 0) cmd--;
					else cmd = 5;
					UART0_send(cmd + '0');
				}
				break;
			}
			/*case 'ñ':{
				ready = 1;
				LED_On();
				break;
			}*/
			default:{
				break;
			}
		}
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
