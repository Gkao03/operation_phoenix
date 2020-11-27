//#include "main.h"
#include "MotorControl.hpp"
#include "USART.hpp"
#include "Timer.hpp"

using namespace ECE477_17;
using namespace Timer;
using namespace USART;

RobotMovementController movementController;

int main(void)
{
	// USART - comment out for testing
	//USART_Init();

	//Initialize other GPIO for shift register and PWM
	//GPIOA_ShiftRegisterPins_Init();

	TIM1_PWM_Init();
	//TIM3_Init();
	//TIM3_Start();

	movementController.SetHighSpeed();
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);

	while(true)
	{
	}

}
