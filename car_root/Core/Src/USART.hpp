#ifndef USART_H
#define USART_H

#include <stm32f4xx.h>
#include <stm32f407xx.h>

extern "C"
{
	//Note: Double check this function name
	void USART1_IRQHandler(void)
	{

	}
}

namespace ECE477_17
{
	namespace USART
	{
		//USART1 GPIOs are PB6 and PB7
		//Setup GPIO + Registers for the USART
		void USART_Initialize(void)
		{
			//USART1 Registers
			USART1->CR1 |= USART_CR1_RXNEIE; //Generate interrupt no receiving data
			USART1->CR1 |= USART_CR1_RE; //Enable receiver
			USART1->CR1 |= USART_CR1_UE; //Enable USART1
			//Enable NVIC interrupt
			NVIC_Enable(USART1_IRQn);
		}
	}
}

#endif
