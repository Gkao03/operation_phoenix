#include "main.h"
#include "MotorControl.hpp"
#include "Timer.hpp"

using namespace ECE477_17;
using namespace Timer;

RobotMovementController movementController;

int main(void)
{
	GPIOA_ShiftRegisterPins_Init();
	TIM1_PWM_Init();
	TIM3_Init();
	//TIM3_Start();

	GPIOD->ODR |= _BS(13); //Turn on orange LED
	movementController.SetLatchPinsToDefaultState();
	//movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);
	//movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
	//Main Program Loop

	while(true)
	{
		for(int i = 0;i < 100000;i++);
		movementController.SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
		movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		for(int i = 0;i < 100000;i++);
		movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);
		movementController.ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();

		GPIOD->ODR ^= _BS(13);
	}

}
