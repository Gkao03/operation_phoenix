#ifndef USART_H
#define USART_H

#include <stm32f4xx.h>
#include <stm32f407xx.h>
#include "Timer.hpp"

using namespace ECE477_17;

extern RobotMovementController movementController;

//USART RX Buffer and control variables
const uint32_t BufferSize = 32;
uint8_t USART1_Buffer_Rx[BufferSize];
uint32_t Rx1_Counter = 0;



extern "C"
{
	void receive(uint8_t *buffer, uint32_t *pCounter)
	{
		// Check RXNE event
		if (USART1->SR & USART_SR_RXNE)
		{
			buffer[*pCounter] = USART1->DR; // Reading DR clears RXNE flag
			(*pCounter)++; // Dereference and update memory value

			if ((*pCounter) >= BufferSize) // Check buffer overflow
			{
				(*pCounter) = 0; // Circular buffer - reset to 0
			}
		}
	}
}

int CommandsPreviouslyReceivedAreEqual(uint8_t buffer[], uint32_t size)
{
	//Grab oldest command. Compare to all until the latest
	uint8_t commandToCompare = buffer[0];

	for(uint32_t i = 1;i < size;i++)
	{
		//Check to see if there are any differences
		//If there are, return 0
		if(commandToCompare != buffer[i]) return 1; //CHANGE to 0!
	}

	//Fall through case is that all commands are equal
	return 1;
}

extern "C"
{
	//Note: Double check this function name - looks good
	void USART1_IRQHandler(void)
	{
		//Magic Hack. Make the initial previous command something that we don't use. The XBEE will be sending an IDLE command to the
		//RC car on startup. This will trigger a movment state update and set the robot movement to IDLE
		static char previousCommand = 'Z';
		char command;

		if(USART1->SR & USART_SR_RXNE)
		{
			command = USART1->DR;
		}

		//Only update if commands are different
		if(previousCommand != command)
		{
			//Parse 'command'
			if(command == 'B')
			{
				movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);
			}
			else if(command == 'A')
			{
				movementController.SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
			}
			else if(command == 'C')
			{
				movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_REVERSE);
			}
			//Rotation commands
			else if(command == 'I')
			{
				movementController.SetCurrentMovementStateAndUpdateMotorDirection(TANK_ROTATE_LEFT);
			}
			else if(command == 'E')
			{
				movementController.SetCurrentMovementStateAndUpdateMotorDirection(TANK_ROTATE_RIGHT);
			}
			//Set Half Speed
			else if(command == 'Q')
			{
				movementController.SetLowSpeed();
			}
			//Set high speed
			else if(command == 'Y')
			{
				movementController.SetHighSpeed();
			}
		}
		//Update previous command
		previousCommand = command;
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
			// GPIO Initialization for USART1 - CHECK IF THIS STILL APPLIES TO STM32F4
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

			// 00 = Input, 01 = Output, 10 = Alternate Function, 11 = Analog
			GPIOB->MODER &= ~(0xF << (2*6)); // Clear mode bits for pin 6 and 7
			GPIOB->MODER |= 0xA << (2*6); // Select Alternate Function mode

			// Alternate Function 7 = USART 1
			GPIOB->AFR[0] |= 0x77 << (4*6); // Set pin 6 and 7 to AF7 - CHECK THIS

			// GPIO Speed: 00 = Low Speed, 01 = Medium Speed, 10 = Fast Speed, 11 = Reserved
			GPIOB->OSPEEDR |= 0xF << (2*6);

			// GPIO push-pull: 00 = No pull-up/down. 01 = Pull up, 10 = Pull down, 11 = Reserved
			GPIOB->PUPDR &= ~ (0xF << (2*6));
			GPIOB->PUPDR |= 0x5 << (2*6); // Select pull-up

			// GPIO Output Type: 0 = push-pull, 1 = open drain
			GPIOB->OTYPER &= ~(0x3<<6);

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
