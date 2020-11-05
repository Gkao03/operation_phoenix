#ifndef TIMER_HPP
#define TIMER_HPP

#include <stm32f4xx.h>

//Bitshift
#define _BS(x) (1 << x)

extern ECE477_17::RobotMovementController movementController;

extern "C"
{
	void TIM3_IRQHandler(void)
	{
		/*
		//Constants
		const uint8_t CLK_RISE = 1;
		const uint8_t CLK_FALL = 0;
		//Keeps track of bit position of latch value to transmit
		static uint8_t ithBitPositionOfLatchValueToTransmit = 0;
		//Keeps track of whether we should set or reset CLK signal.
		static uint8_t signalCLKRiseOrFall = CLK_RISE;
		//Keeps track of whether we have just started a transmission
		static uint8_t transmitting = 0;

		//This checks whether we have just attempted to start a shift reg. transmission
		//If we have, we need time for the latch transmission to start (wait for signals to go up)
		if ( movementController.latchTriggerUpdateStatus == ECE477_17::LATCH_STATUS_TRIGGER_UPDATE && transmitting)
		{
			movementController.LatchTransmitStart();
			//Set transmitting to 1
			transmitting = 1;

			//Zero out SR so we indicate we have finished the interrupt routine (else get stuck here.... forever!)
			TIM3->SR = 0;
			return;
		}
		//Update signalCLKRiseOrFall, take a particular action
		//Set CLK to LOW
		if( signalCLKRiseOrFall == CLK_RISE && movementController.latchTriggerUpdateStatus == ECE477_17::LATCH_STATUS_TRIGGER_UPDATE)
		{
			//Set CLK LOW so we can shift something in
			ECE477_17::SHIFT_REGISTER_GPIO->ODR &= ~ECE477_17::SHIFT_REGISTER_CLK;

			signalCLKRiseOrFall = CLK_FALL;
		}
		//Set CLK to HIGH
		else if ( signalCLKRiseOrFall == CLK_FALL && movementController.latchTriggerUpdateStatus == ECE477_17::LATCH_STATUS_TRIGGER_UPDATE)
		{
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback(ithBitPositionOfLatchValueToTransmit, transmitting);
			//Set CLK HIGH
			ECE477_17::SHIFT_REGISTER_GPIO->ODR |= ECE477_17::SHIFT_REGISTER_CLK;

			signalCLKRiseOrFall = CLK_RISE;
		}

		//Turn on LED so we know this IRQ has been called
		//Safe to remove. Just for debugging.
		GPIOD->ODR ^= _BS(13); //Turn on orange LED
		*/



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

			//Select TIM1 Channels for PE9,11,13,14
			GPIOE->AFR[1] 	= ( _BS(4) | _BS(12) | _BS(20) | _BS(24) ); //Choose AF's
			GPIOE->MODER  	= ( _BS(19) | _BS(23) | _BS(27) | _BS(29) ); //Set to AF mode
			GPIOE->OSPEEDR 	= ( _BS(19) | _BS(23) | _BS(27) | _BS(29) ); //Set to high speed
			GPIOE->PUPDR	= 0;

			//Setup TIM1
			//Recall: System runs at 25MHz, and we want ~490   Hz???
			TIM1->ARR = 3000-1;
			TIM1->PSC = 17-1;
			//TIM1->DIER |= _BS(0); //Enable UIE
			//TIM1->EGR  |= _BS(0); //Generate update
			TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //Set PWM Mode
			TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
			TIM1->CCR1	= 1500;
			TIM1->CCR2 	= 1500;
			TIM1->CCR3	= 1500;
			TIM1->CCR4	= 1500;

			//Duty Cycle for CCRX = ARR / CCRX

			TIM1->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
			TIM1->BDTR  |= TIM_BDTR_MOE;

			TIM1->CR1  |= TIM_CR1_CEN; //Count enable
		}
		//Start PWM. This only need sto be called after StopPWM(...) Since CEN is on after TIM1_PWM_INIT()
		void TIM1_StartPWM(void) { TIM1->CR1 |= TIM_CR1_CEN; }
		//Stop PWM.
		void TIM1_StopPWM(void) { TIM1->CR1 &= ~TIM_CR1_CEN; }

		//Stop PWM, change CCR1, Start PWM
		void TIM1_ChangePWM(int tim_channel, float dutyCycle)
		{
			TIM1_StopPWM();

			uint32_t newCCRX = dutyCycle * TIM1->ARR;

			if(tim_channel == 1)
			{
				TIM1->CCR1 = newCCRX;
			}
			if(tim_channel == 2)
			{
				TIM1->CCR2 = newCCRX;
			}
			if(tim_channel == 3)
			{
				TIM1->CCR3 = newCCRX;
			}
			if(tim_channel == 4)
			{
				TIM1->CCR4 = newCCRX;
			}

			TIM1_StartPWM();
		}

		//TIM3 - Interrupt generator for shift register control
		void TIM3_Init(void)
		{
			//Clock TIM3
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
			//Setup TIM1
			//42 HZ transmittion 2000-1, 2000-1
			//For seeing on scope
			TIM3->ARR 	= 500-1;
			TIM3->PSC 	= 500-1;
			TIM3->DIER	|= TIM_DIER_UIE;
			TIM3->SR	= 0;
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//Uncomment to enable IRQ
			//NVIC->ISER[0] |= _BS(TIM3_IRQn);
			//__enable_irq();
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		}
		//Start + Stop routines
		void TIM3_Start(void){ TIM3->CR1 |= TIM_CR1_CEN; }
		void TIM3_Stop(void){ TIM3->CR1 &= ~TIM_CR1_CEN; }

	}
}

#endif
