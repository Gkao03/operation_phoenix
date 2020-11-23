#ifndef USART_H
#define USART_H

#include <stm32f4xx.h>
#include <stm32f407xx.h>
#include "Timer.hpp"

//USART RX Buffer and control variables
#define BufferSize 32
uint8_t USART1_Buffer_Rx[BufferSize];
uint32_t Rx1_Counter = 0;
int test = 0;

//Command 'Shift Register'
#define COMMAND_QUEUE_SIZE 3
//IDLE by default
uint8_t commands_previously_received[COMMAND_QUEUE_SIZE] = {'I', 'I', 'I'};

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

//Helper function. Shift elements of previously received commands
void ShiftInNewCommandToCommandShiftRegister(uint8_t command)
{
	for(int i = 0;i < COMMAND_QUEUE_SIZE-1;i++)
	{
		commands_previously_received[i] = commands_previously_received[i+1];
	}

	commands_previously_received[COMMAND_QUEUE_SIZE-1] = command;
}

int CommandsPreviouslyReceivedAreEqual(void)
{
	//Grab oldest command. Compare to all until the latest
	uint8_t commandToCompare = commands_previously_received[0];

	for(int i = 1;i < COMMAND_QUEUE_SIZE;i++)
	{
		//Check to see if there are any differences
		//If there are, return 0
		if(commandToCompare != commands_previously_received[i]) return 0;
	}

	//Fall through case is that all commands are equal
	return 1;
}

extern "C"
{
	//Note: Double check this function name - looks good
	void USART1_IRQHandler(void)
	{
		//Last command received - at startup this will be IDLE
		//Further invocations of this function will be whatever the most recent command was form the previous call
		static char prevCommand = commands_previously_received[3];

		//Grab data from RX buffer
		receive(USART1_Buffer_Rx, &Rx1_Counter);

		//Archive the command
		char command = (char) USART1_Buffer_Rx[Rx1_Counter-1];

		//Use helper function to shift in most recent command to shift register
		ShiftInNewCommandToCommandShiftRegister(command);

		//Decide if most recent three commands are the same
		if(CommandsPreviouslyReceivedAreEqual() == 0) return;

		//Parse 'command'
		if(command == 'B')
		{
			movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}
		else if(command == 'A')
		{
			movementController.SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}
		else if(command == 'C')
		{
			movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_REVERSE);
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}
		//Rotation commands
		else if(command == 'I')
		{
			movementController.SetCurrentMovementStateAndUpdateMotorDirection(TANK_ROTATE_LEFT);
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}
		else if(command == 'E')
		{
			movementController.SetCurrentMovementStateAndUpdateMotorDirection(TANK_ROTATE_RIGHT);
			movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}
		//Set Half Speed
		else if(command == 'Q')
		{
			Timer::TIM1_ChangePWM(1250);
		}
		//Set high speed
		else if(command == 'Y')
		{
			Timer::TIM1_ChangePWM(2500);
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
