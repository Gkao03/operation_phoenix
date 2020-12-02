//#include "main.h"
#include "MotorControl.hpp"
#include "USART.hpp"
#include "Timer.hpp"

using namespace ECE477_17;
using namespace Timer;
using namespace USART;

RobotMovementController movementController;
uint32_t tim3_counter_limit = 57; //50 * 1/100 = 0.5 seconds
//TIM3 wait enable variable
extern bool doCount;

//Wait function, it is simply a for loop that iterates k times
void fools_wait(uint32_t k){ for(uint32_t i = 0;i < k;i++); }

int main(void)
{
	// USART - comment out for testing
	USART_Init();
	//Start PWM Timer
	TIM1_PWM_Init();
	//Set low speed
	movementController.SetHighSpeed();
	movementController.SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
	//Setup TIM3
	TIM3_Init();
	//Turn off LED to start
	GPIOD->ODR &= ~_BS(14);

	//Infinite loop
	while(true){}
}
