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
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define SYSCFG_EXTICR1_EXTI0  ((uint16_t)0x000F) /*!< EXTI 0 configuration line 3301 */
#define SYSCFG_EXTICR1_EXTI0_PA 	((uint16_t)0x0000) /*!< PA[0] pin line 3309 */
#define EXTI_PR_PR0         ((uint32_t)0x00000001)        /*!< Pending bit 0 line 1972 */

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
		/* MODER0[1:0] corresponds to bit 0, MODER3[1:0] is to bit 3 of PA.3. MODER3[1:0] are bits 6&7 
		in GPIOB->MODER register, mask 00 for input. LED is PB.6 so we use MODER6[1:0] involving bits
		12 & 13. output 01 so |= 0x01. Use &= ~0b11 or &= ~0x03 to clear */
		//GPIOB->MODER &= ~(0x03<<(2*6));		// Clear mode bits. Uses 2 bits to select mode input, output, alternate function & analog
		//GPIOB->MODER |= 0x01<<(2*6);			// Set pin PB.6 as digital output
		GPIOB->MODER &= ~(0x03 << 12);			// Clear mode bits
		GPIOB->MODER |= 0x01 << 12;					// Set pin PB.6 as digital output
		
		// Bits 15:0 OTy: Port x configuration bits (y = 0..15). OT6 for pin PB.6 as push-pull 0x0, open-drain 0x1
		GPIOB->OTYPER &= ~(0x1<<6);					// Set PB.6 pin as push-pull output type
	
		// Set pin 6 as alternative function 2 (TIM4)
		//GPIOB->AFR[0] |= 0X2 << (4*6);
	
		// Set IO output speed
		GPIOB->OSPEEDR &= ~(0x03<<(2*6));
		GPIOB->OSPEEDR |= 0x01<<(2*6);
	
		// Set IO no pull-up pull-down
		GPIOB->PUPDR &= ~(0x03<<(2*6));	
}

void GPIO_Init_PortA(){
		/* MODER0[1:0] corresponds to bit 0, MODER3[1:0] is to bit 3 of PA.3. MODER3[1:0] are bits 6&7 
		in GPIOA->MODER register, mask 00 for input. Pushbutton is PA.0 so we use MODER0[1:0]
		|= 0x00. Use &= ~0x03 to clear */
		//GPIOA->MODER &= ~(0x03);					// Set pin PA.0 as digital input
		GPIOA->MODER &= ~(3UL << 0);
	
		// Bits 15:0 OTy: Port x configuration bits (y = 0..15). OT0 for pin PA.0 as push-pull 0x0, open-drain 0x1
		GPIOA->OTYPER &= ~(0x1<<0);					// Set PA.0 pin as push-pull output type
	
		// Set IO output speed
		GPIOA->OSPEEDR &= ~(0x03);
		GPIOA->OSPEEDR |= 0x01;
	
		// Set IO no pull-up pull-down
		//GPIOA->PUPDR &= ~(0x03);
		GPIOA->PUPDR &= ~3U << 0;
		GPIOA->PUPDR |= 2U << 0;
}	

void EXTI_Init(){
		// Enable SYSCFG clock
		RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;
	
		// EXTIx[3:0]: EXTI x configuration PA[x]=0000, PB[x]=0001, PC[x]=0010, PD[x]=0011, page 220 rm0038
		SYSCFG->EXTICR[(BUTTON_PIN/4)] &= ~SYSCFG_EXTICR1_EXTI0;		// EXTI0 configuration
		SYSCFG->EXTICR[(BUTTON_PIN/4)] |= SYSCFG_EXTICR1_EXTI0_PA;	// PA[0] pin configuration
	
		// Select trigger edge for EXTI0. 0 = rising edge trigger disabled
		// 1 = rising edge trigger enabled. line 1897 stm32l1xx.h
		// #define  EXTI_RTSR_TR0     ((uint32_t)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
		EXTI->RTSR |= EXTI_RTSR_TR0;				// Set to rising edge 
		EXTI->IMR |= EXTI_IMR_MR0;					// Enable EXTI0 interrupt
		
		//EXTI->IMR  |=  (1 << BUTTON_PIN);
		//EXTI->RTSR &= ~(1 << BUTTON_PIN);
		//EXTI->FTSR |=  (1 << BUTTON_PIN);
	
		NVIC_SetPriority(EXTI0_IRQn, 0x03);		// Set EXTI0 priority 1(low priority)
		NVIC_EnableIRQ(EXTI0_IRQn);					// Enable EXTI0 interrupt
}

void TIM4_Clock_Enable(){
		// Enable clock to TIM4
		RCC->APB1ENR	|= RCC_APB1ENR_TIM4EN;
	  // Enable the NVIC interrupt for TIM4
		NVIC_SetPriority(TIM4_IRQn, 0x03);
		NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQ_handler(void){
		// Handle a timer 'update' interrupt event
		if(TIM4->SR & TIM_SR_UIF){
				TIM4->SR &= ~(TIM_SR_UIF);
				// Toggle the LED output pin.
				GPIOB->ODR ^= (1<<6);
		}
}

void EXTI0_IRQHandler(void){				// Defined & enumerated from interrupt vector table
		if(EXTI->PR & (1<<0)){					// Check EXTI0 interrupt flag
				GPIOB->ODR ^= 1<<LED_PIN;		// Toggle PB.6 LED
				//EXTI->PR |= (1<<0);					// Clear EXTI0 pending interrupt by set to 1
				EXTI->PR |= EXTI_PR_PR0; 		// Clear EXTI0 pending interrupt by set to 1
		}
		GPIOB->ODR |= 1<<LED_PIN;	
}

int main(void){
		GPIO_Clock_Enable();			// Clocks for both push-button(PA.0) and led(PB.6)
		GPIO_Init_PortB();
		//GPIOB->ODR |= 1<<LED_PIN;
		GPIO_Init_PortA();

		EXTI_Init();
	
		// Input value stored in bit 0 of GPIOA->IDR
		// Infinite loop. Interrupt method.
		while(1);
}
