#ifndef _NSC_MAIN_H
#define _NSC_MAIN_H

#include <stdint.h>

/* Standard STM32L1xxx driver headers */
#include "stm32l1xx.h"

/* STM32L1xx Discovery Kit:
    - USER Pushbutton: connected to PA0 (GPIO Port A, PIN 0), CLK RCC_AHBENR_GPIOAEN
    - RESET Pushbutton: connected RESET
    - GREEN LED: connected to PB7 (GPIO Port B, PIN 7), CLK RCC_AHBENR_GPIOBEN 
    - BLUE LED: connected to PB6 (GPIO Port B, PIN 6), CLK RCC_AHBENR_GPIOBEN
    - Linear touch sensor/touchkeys: PA6, PA7 (group 2),  PC4, PC5 (group 9),  PB0, PB1 (group 3)
*/

/* stm32l1.h  line 815. GPIO_TypeDef pointer to GPIO_BASE address 0x40020400 */
#define GPIOB               ((GPIO_TypeDef *) 0x40020400)
//#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define RCC_AHBENR_GPIOBEN  ((uint32_t)0x00000002)        /*!< GPIO port B clock enable */
#define RCC_AHBENR_GPIOAEN  ((uint32_t)0x00000001)        /*!< GPIO port A clock enable */
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)	/* line 794. EXTI pointer to this structure */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)	/* line 814 */

#define LED_PIN			(6)
#define BUTTON_PIN 	(0)

#endif	// Prevent the file from being included more than once.

void GPIO_Clock_Enable(){
		// Enable GPIO port B clock
		RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	//RCC->AHBENR	|= 0x00000002;
		// Enable GPIO port A clock
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	//RCC->AHBENR	|= 0x00000001;
}

void GPIO_Init_PortB(){
		// Set pin 6 as general-purpose output
		GPIOB->MODER &= ~(0x03<<(2*6));		// Mode mask
		GPIOB->MODER |= 0x01<<(2*6);			// Set pin 6 as digital output
	
		// Set output type as push-pull
		GPIOB->OTYPER &= ~(1<<6);
	
		// Set pin 6 as alternative function 2 (TIM4)
		//GPIOB->AFR[0] |= 0X2 << (4*6);
	
		// Set IO output speed
		GPIOB->OSPEEDR &= ~(0x03<<(2*6));
		GPIOB->OSPEEDR |= 0x01<<(2*6);
	
		// Set IO no pull-up pull-down
		GPIOB->PUPDR &= ~(0x03<<(2*6));	
}

void GPIO_Init_PortA(){
		GPIOA->MODER &= ~(0x03);					// Set pin 0 as digital input
	
		// Set output type as push-pull
		GPIOA->OTYPER &= ~(0x1);
	
		// Set IO output speed
		GPIOA->OSPEEDR &= ~(0x03);
		GPIOA->OSPEEDR |= 0x01;
	
		// Set IO no pull-up pull-down
		GPIOA->PUPDR &= ~(0x03);
	
		//GPIOA->MODER  &= ~(0x3 << (BUTTON_PIN*2));
		//GPIOA->PUPDR  &= ~(0x3 << (BUTTON_PIN*2));
		//GPIOA->PUPDR  |=  (0x1 << (BUTTON_PIN*2));
}	

void EXTI_Init(){
		// Enable SYSCFG clock
		RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;
	
		// EXTIx: PA[x]=0000, PB[x]=0001, PC[x]=0010, PD[x]=0011
		// Clear bits and select PA.0 for EXTI line 0
		SYSCFG->EXTICR[0] &= ~(0x000F);			// Connect EXTI line 0 to GPIO port A pin 0
	
		// Select trigger edge for EXTI0. 0 = rising edge trigger disabled
		// 1 = rising edge trigger enabled. line 1897 stm32l1xx.h
		// #define  EXTI_RTSR_TR0     ((uint32_t)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
		EXTI->RTSR | EXTI_RTSR_TR0;					// Set to rising edge 
	
		EXTI->IMR |= EXTI_IMR_MR0;					// Enable EXTI0 interrupt
	
		NVIC_SetPriority(EXTI0_IRQn, 1);		// Set EXTI0 priority 1(low priority)
		NVIC_EnableIRQ(EXTI0_IRQn);					// Enable EXTI0 interrupt
	
		//GPIOB->ODR |= 1<<LED_PIN;
}

void TIM4_Clock_Enable(){
		// Enable clock to TIM4
		RCC->APB1ENR	|= RCC_APB1ENR_TIM4EN;
	  // Enable the NVIC interrupt for TIM4
		NVIC_SetPriority(TIM4_IRQn, 0x03);
		NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQ_handler(void) {
		// Handle a timer 'update' interrupt event
		if(TIM4->SR & TIM_SR_UIF){
				TIM4->SR &= ~(TIM_SR_UIF);
				// Toggle the LED output pin.
				GPIOB->ODR ^= (1<<6);
		}
}

void EXTI0_IRQHandler(void){				// Defined & enumerated from interrupt vector table
		if(EXTI->PR & (1<<0)){					// Check EXTI0 interrupt flag
				GPIOB->ODR ^= 1<<LED_PIN;					// Toggle PB.6 LED
				EXTI->PR |= (1<<0);					// Clear EXTI0 pending interrupt
		}
		GPIOB->ODR |= 1<<LED_PIN;
}

int main(void){
		GPIO_Clock_Enable();			// Clocks for both push-button(PA.0) and led(PB.6)
		GPIO_Init_PortB();
		//GPIOB->ODR |= 1<<LED_PIN;
		GPIO_Init_PortA();

		//EXTI_Init();
	
		uint8_t button_down = 0;	// Status variable for toggling led
		// Infinite loop. Polling method
		while(1){
			  // Invert the IDR register since '0' means 'pressed'
				uint32_t idr_val = ~GPIOA->IDR;	// Input value stored in bit 0 of GPIOA->IDR
				if(idr_val & (1<<BUTTON_PIN)){
						// The button is pressed; if it was not already
						// pressed, change the LED state or toggle
						if (!button_down) {
								GPIOB->ODR ^= (1<<LED_PIN);
						}
						button_down = 1;
				}
				else {
						button_down = 0;
				}
		}
}
