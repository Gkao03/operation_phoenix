#include "main.h"
#include "MotorControl.hpp"
#include "USART.hpp"
#include "Timer.hpp"

using namespace ECE477_17;
using namespace Timer;
using namespace USART;

RobotMovementController movementController;

int main(void)
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

	// USART
	USART_Init();



	GPIOA_ShiftRegisterPins_Init();
	TIM1_PWM_Init();
	TIM3_Init();
	//TIM3_Start();

	GPIOD->ODR |= _BS(13); //Turn on orange LED

	movementController.SetLatchPinsToDefaultState();
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
	movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
	//Main Program Loop

	while(true);

}
