//#include "main.h"
#include "MotorControl.hpp"
#include "ShiftRegisterUpdateStateMachine.hpp"
#include "USART.hpp"
#include "Timer.hpp"

using namespace ECE477_17;
using namespace Timer;
using namespace USART;

RobotMovementController movementController;

ShiftRegisterUpdateStateMachine shiftRegisterStateMachine;

int main(void)
{
	// USART - comment out for testing
	//USART_Init();

	//Initialize other GPIO for shift register and PWM
	//GPIOA_ShiftRegisterPins_Init();

	TIM1_PWM_Init();
	//TIM3_Init();
	//TIM3_Start();

	//Set latch pins to default state so we don't update them by accident during startup

	/*movementController.SetLatchPinsToDefaultState();

	//Wait before setting to IDLE to avoid any odd behavior with pins
	for(int i = 0;i < 20000;i++);

	//Default mode on powerup is IDLE.
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);
	//Proper way to start transmitting
	if(shiftRegisterStateMachine.ReadyToUpdateShiftRegister()) shiftRegisterStateMachine.beginTransmit = true;
	//Wait to finish
	while(shiftRegisterStateMachine.IsCurrentlyUpdatingShiftRegister());
	*/
	while(true)
	{
	}

}
