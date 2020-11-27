//#include "main.h"
#include "MotorControl.hpp"
#include "USART.hpp"
#include "Timer.hpp"

using namespace ECE477_17;
using namespace Timer;
using namespace USART;

RobotMovementController movementController;

//Wait function, it is simply a for loop that iterates k times
void fools_wait(uint32_t k){ for(uint32_t i = 0;i < k;i++); }

int main(void)
{
	// USART - comment out for testing
	USART_Init();

	//Start PWM Timer
	TIM1_PWM_Init();

	//Set LOW motor speed
	movementController.SetLowSpeed();
	//Set motor movement scheme
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
	//Wait a little bit
	fools_wait(2000000);
	//Step through all motor movements
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_FORWARD);
	//Wait a little bit
	fools_wait(2000000);
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(FULL_REVERSE);
	//Wait a little bit
	fools_wait(2000000);
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(TANK_ROTATE_LEFT);
	//Wait a little bit
	fools_wait(2000000);
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(TANK_ROTATE_RIGHT);
	//Wait a little bit
	fools_wait(2000000);

	//Infinite loop
	while(true){}
}
