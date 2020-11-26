#ifndef TIMER_HPP
#define TIMER_HPP

#include <stm32f4xx.h>

//Bitshift
#define _BS(x) (1 << x)

extern ECE477_17::RobotMovementController movementController;
extern ECE477_17::ShiftRegisterUpdateStateMachine shiftRegisterStateMachine;

extern "C"
{
	void TIM3_IRQHandler(void)
	{

		//Update state machine as necessary
		shiftRegisterStateMachine.UpdateState(movementController.updatedLatchValueToTransmit);

		//Zero out SR so we indicate we have finished the interrupt routine (else get stuck here.... forever!)
		TIM3->SR = 0;
	}
}

namespace ECE477_17
{
	namespace Timer
	{
		void GPIOA_ShiftRegisterPins_Init(void)
		{
			RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOAEN;

			GPIOA->MODER 	|= ( _BS(2) | _BS(4) | _BS(6) | _BS(8) ); //Set output mode for GPIOA 1-4
			GPIOA->OSPEEDR 	|= ( _BS(2) | _BS(3) | _BS(4) | _BS(5) | _BS(6) | _BS(7) | _BS(8) | _BS(9) );
			GPIOA->ODR   	= 0; //Everything is 0 by default

			//Enable GPIOD for blue LED
			RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIODEN;
			GPIOD->MODER 	|= _BS(28) | _BS(26); //Set GPIOD 14,13 to output mode
			GPIOD->ODR   	= 0; //Everything is 0 by default
			GPIOD->ODR 		|= _BS(14);
			GPIOD->PUPDR	= 0;
			GPIOD->OSPEEDR  |= _BS(28) | _BS(26);
		}

		//Initialize TIM1 and its 4 Channels
		void TIM1_PWM_Init(void)
		{
			//Enable Clock to TIM1, GPIOE (PWM GPIO pins)
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

			//Select TIM1 Channels for PE9 and NOT 11,13,14
			GPIOE->AFR[1] 	= ( _BS(4) | _BS(12) | _BS(20) | _BS(24) ); //Choose AF's
			GPIOE->MODER  	= ( _BS(19) | _BS(23) | _BS(27) | _BS(29) ); //Set to AF mode
			GPIOE->OSPEEDR 	= ( _BS(19) | _BS(23) | _BS(27) | _BS(29) ); //Set to high speed
			GPIOE->PUPDR	= 0;

			//Setup TIM1
			//Recall: System runs at 25MHz, and we want 5 kHz frequency
			TIM1->ARR = 20-1;
			TIM1->PSC = 250-1;
			//TIM1->DIER |= _BS(0); //Enable UIE
			//TIM1->EGR  |= _BS(0); //Generate update
			TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //Set PWM Mode

			TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

			//To move forward, set CCR2 to 0 and CCR1 to < 20.
			// To move in reverse, set CCR1 to 0 and CCR2 to < 20.
			// DO NOT SET BOTH CCRs to non-zero values.
			// NEED TO SELECT WHETHER Channels 1 and 2 correspond to left or right of vehicle
			// Manually toggling these right now, need to make them automatically switch
			TIM1->CCR1	= 19;
			TIM1->CCR2 	= 0;

			//To move forward, set CCR4 to 0 and CCR3 to < 20.
			// To move in reverse, set CCR3 to 0 and CCR4 to < 20.
			// DO NOT SET BOTH CCRs to non-zero values.
			// Manually toggling these right now, need to make them automatically switch
			// NEED TO SELECT WHETHER Channels 3 and 4 correspond to left or right of vehicle
			TIM1->CCR3	= 19;
			TIM1->CCR4	= 0;

			//Duty Cycle is CCRX / ARR
			TIM1->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
			TIM1->BDTR  |= TIM_BDTR_MOE;

			TIM1->CR1  |= TIM_CR1_CEN; //Count enable
		}

		//Start PWM. This only needs to be called after StopPWM(...) Since CEN is on after TIM1_PWM_INIT()
		void TIM1_StartPWM(void) { TIM1->CR1 |= TIM_CR1_CEN; }
		//Stop PWM.
		void TIM1_StopPWM(void) { TIM1->CR1 &= ~TIM_CR1_CEN; }

		//Stop PWM, change CCR1, Start PWM
		void TIM1_ChangePWM(uint32_t newCCRx)
		{
			TIM1->CCR1 = newCCRx;
			TIM1->CCR2 = newCCRx;
			TIM1->CCR3 = newCCRx;
			TIM1->CCR4 = newCCRx;
		}

		//TIM3 - Interrupt generator for shift register control
		void TIM3_Init(void)
		{
			//Clock TIM3
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
			//Setup TIM3
			//650 HZ update 200, 200. 1khz for 125, 200. 1/3 Hz for ARR = 3000, PSC = 25000
			//For seeing on scope
			TIM3->ARR 	= 3000-1;
			TIM3->PSC 	= 25000-1;
			TIM3->DIER	|= TIM_DIER_UIE;
			TIM3->SR	= 0;
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//Uncomment to enable IRQ
			NVIC->ISER[0] |= _BS(TIM3_IRQn);
			__enable_irq();
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		}
		//Start + Stop routines
		void TIM3_Start(void){ TIM3->CR1 |= TIM_CR1_CEN; }
		void TIM3_Stop(void){ TIM3->CR1 &= ~TIM_CR1_CEN; }

	}
}

#endif
