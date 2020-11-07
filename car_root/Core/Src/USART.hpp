#ifndef USART_H
#define USART_H

#include <stm32f4xx.h>
#include <stm32f407xx.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

#define BufferSize 32
uint8_t USART1_Buffer_Rx[BufferSize];
uint32_t Rx1_Counter = 0;
int test = 0;

using namespace ECE477_17;

extern RobotMovementController movementController;

extern "C"
{
	void receive(uint8_t *buffer, uint32_t *pCounter)
	{
		// Check RXNE event
		if (USART1->SR & USART_SR_RXNE)
		{
			test = 1;
			buffer[*pCounter] = USART1->DR; // Reading DR clears RXNE flag
			(*pCounter)++; // Dereference and update memory value
			if ((*pCounter) >= BufferSize) // Check buffer overflow
			{
				(*pCounter) = 0; // Circular buffer
			}

		}
	}
}

extern "C"
{
	//Note: Double check this function name - looks good
	void USART1_IRQHandler(void)
	{
		static char prevCommand = 'i';

		receive(USART1_Buffer_Rx, &Rx1_Counter);

		char command = (char) USART1_Buffer_Rx[Rx1_Counter-1];

		if(command != prevCommand && command == 'g')
		{
			prevCommand = command;
			movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}
		if(command != prevCommand && command == 'i')
		{
			prevCommand = command;
			movementController.SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}
	}
}

namespace ECE477_17
{
	namespace USART
	{
		//USART1 GPIOs are PB6 and PB7 - PCLK2 = 16 MHz
		//Setup GPIO + Registers for the USART
		void USART_Init(void)
		{
			// Disable USART
			USART1->CR1 &= ~USART_CR1_UE;

			// Set data length to 8 bits
			// 00 = 8 data bits, 01 = 9 data bits, 10 = 7 data bits
			USART1->CR2 &= ~USART_CR2_STOP;

			// Set parity control as no parity
			// 0 = no parity
			// 1 = parity enabled (then, program PS bit to select Even or Odd parity)
			USART1->CR1 &= ~USART_CR1_PCE;

			// Oversampling by 16
			// 0 = oversampling by 16, 1 = oversampling by 8
			USART1->CR1 &= ~USART_CR1_OVER8;

			// Set Baud Rate to 115200 using APB frequency (16 MHz) - NEED TO UPDATE!
			USART1->BRR = 0x8B;

			//USART1 Registers
			USART1->CR1 |= USART_CR1_RXNEIE; //Generate interrupt no receiving data
			USART1->CR1 |= USART_CR1_RE; //Enable receiver
			USART1->CR1 |= USART_CR1_UE; //Enable USART1

			//Enable NVIC interrupt
			NVIC_EnableIRQ(USART1_IRQn);
			//NVIC->ISER[]??

			// Verify USART is ready for reception
			// REACK: Receive enable acknowledge flag. Hardware sets or resets it.
			//while (!(USART1->SR & USART_SR_RXNE));
		}
	}
}

#endif
