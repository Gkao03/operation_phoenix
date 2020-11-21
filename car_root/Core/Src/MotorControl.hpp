/*
 * MotorControl.h
 *
 *  Created on: Oct 5, 2020
 *      Author: gorshborsh
 */

#ifndef SRC_MOTORCONTROL_HPP_
#define SRC_MOTORCONTROL_HPP_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
//#include "misc.h"

//Bitshift
#define _BS(x) (1 << x)

namespace ECE477_17
{
	//Declare constants
	//Motor GPIO Pins. NOTE: These are for GPIOA.
	const auto PWM_GPIO						= GPIOE;
	const auto FRONT_LEFT_MOTOR_PWM_PIN 	= GPIO_PIN_9;
	const auto FRONT_RIGHT_MOTOR_PWM_PIN 	= GPIO_PIN_11;
	const auto BACK_LEFT_MOTOR_PWM_PIN 		= GPIO_PIN_13;
	const auto BACK_RIGHT_MOTOR_PWM_PIN 	= GPIO_PIN_14;
	//Shift Register GPIO Interface
	//Shift Register is bridge between motor control L293D chips and STM32F4.
	const auto SHIFT_REGISTER_GPIO			= GPIOA;
	const auto SHIFT_REGISTER_LATCH 		= _BS(1);
	const auto SHIFT_REGISTER_DATA			= _BS(2);
	const auto SHIFT_REGISTER_CLK			= _BS(3);
	const auto SHIFT_REGISTER_ENABLE		= _BS(4);
	//From Adafruit Github
	// Arduino pin names for interface to 74HCT595 latch
	//#define MOTORLATCH 12
	//#define MOTORCLK 4
	//#define MOTORENABLE 7
	//#define MOTORDATA 8

	//Lookup table - our pins to boards
	//GPIOA1 -> Pin 12
	//GPIOA2 -> Pin 8
	//GPIOA3 -> Pin 4
	//GPIOA4 -> Pin 7

	/*
	#define MOTOR1_A 2
	#define MOTOR1_B 3
	#define MOTOR2_A 1
	#define MOTOR2_B 4
	#define MOTOR4_A 0
	#define MOTOR4_B 6
	#define MOTOR3_A 5
	#define MOTOR3_B 7
	*/
	const uint8_t MOTOR_FL_A					=  _BS(2);
	const uint8_t MOTOR_FL_B					=  _BS(3);
	const uint8_t MOTOR_BL_A					=  _BS(1);
	const uint8_t MOTOR_BL_B					=  _BS(4);
	const uint8_t MOTOR_FR_A					=  _BS(5);
	const uint8_t MOTOR_FR_B					=  _BS(7);
	const uint8_t MOTOR_BR_A					=  _BS(0);
	const uint8_t MOTOR_BR_B					=  _BS(6);
	const uint8_t LATCH_STATUS_TRIGGER_UPDATE			= 1;
	const uint8_t LATCH_STATUS_NO_UPDATE				= 0;

	//Enum that describes whether a particular motor should drive forwards or backwards
	enum MOTOR_DIRECTION{ FORWARD, REVERSE, NONE };
	//Enum that describes how the robot should move. This determines the MOTOR_DIRECTION of each motor
	enum ROBOT_MOVEMENT_STATE{ IDLE, FULL_FORWARD, FULL_REVERSE, TANK_ROTATE_LEFT, TANK_ROTATE_RIGHT, SOFT_ROTATE_LEFT, SOFT_ROTATE_RIGHT };
	enum MOTOR_ID {FRONT_LEFT, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT};

	//Struct for controlling motors
	struct RobotMovementController
	{
		//Constructor
		RobotMovementController(void)
		{
			//Set all to forward by default
			this->backright_motor_dir 	= NONE;
			this->backleft_motor_dir 	= NONE;
			this->frontright_motor_dir 	= NONE;
			this->frontleft_motor_dir 	= NONE;
			//Set movement state to Idle
			this->current_movement_state = IDLE;
			this->updatedLatchValueToTransmit = 0x00;
		}

		//Destructor (unused)
		~RobotMovementController(void){}

		//This should be called in main program loop or in a timer callback proc
		//Updates PWM Signals for motors
		void SetMotorSpeed(MOTOR_ID motor, int speed)
		{

		}

		//Assign new motor values to shift register
		//int& ithBitPosition...Transmit
		// Incremented by 1 everytime the callback is called. Once it hits 8, the internal flag 'latchTriggerUpdateStatus' is rest, signalling for this function to not send
		// new values to the shift register
		void ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback(void)
		{
			//FIRST - Do not update the shift register state if the latch value has not changed since this function was last called
			//Need to store value of previousLatchValue - make it static
			static uint8_t previousLatchValue = 0;
			if(previousLatchValue == this->updatedLatchValueToTransmit) return;

			//Set LATCH low
			SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_LATCH;

			//Wait
			for(unsigned int i = 0; i < 10000; i++);

			//Transmit 8 bits to serial register
			for(int i = 0;i < 8; i++)
			{
				//Set CLK low
				SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_CLK;
				//Wait
				for(unsigned int i = 0; i < 10000; i++);

				//Get the i'th bit position of latchValue
				//uint8_t ithBitValueOfLatchValueToTransmit = (this->updatedLatchValueToTransmit >> (7-i)) & 0x01;
				uint8_t ithBitValueOfLatchValueToTransmit = updatedLatchValueToTransmit & _BS((7-i));

				//Transmit the bits of the latchValueToTransmit
				if (ithBitValueOfLatchValueToTransmit)
				{
					SHIFT_REGISTER_GPIO->ODR |=  SHIFT_REGISTER_DATA;
				}
				else
				{
					SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_DATA;
				}

				for(unsigned int i = 0; i < 10000; i++);

				SHIFT_REGISTER_GPIO->ODR |= SHIFT_REGISTER_CLK;
				//Wait
				for(unsigned int i = 0; i < 10000; i++);
			}

			//Set LATCH high
			SHIFT_REGISTER_GPIO->ODR |= SHIFT_REGISTER_LATCH;

			//Update previous latch value
			previousLatchValue = this->updatedLatchValueToTransmit;
		}

		void LatchTransmitStart(void)
		{
			//Set ENABLE to HIGH
			//Set LATCH to LOW
			//Set MOTORDATA LOW
			SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_LATCH;

		}

		void LatchTransmitHalt(void)
		{
			SHIFT_REGISTER_GPIO->ODR |=  SHIFT_REGISTER_LATCH;
			SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_DATA;
		}


		void SetLatchPinsToDefaultState(void)
		{
			SHIFT_REGISTER_GPIO->ODR |=  SHIFT_REGISTER_LATCH;
			//This needs to always be low
			SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_ENABLE;
			SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_DATA;
			SHIFT_REGISTER_GPIO->ODR |=  SHIFT_REGISTER_CLK;

			this->SetCurrentMovementStateAndUpdateMotorDirection(IDLE);
			this->ShiftRegisterAssignMotorEnableDirectionValues_TIM3_InterruptCallback();
		}


		//Set current movement state
		void SetCurrentMovementStateAndUpdateMotorDirection(ROBOT_MOVEMENT_STATE newState)
		{
			//Store the new state
			this->current_movement_state = newState;
			//Update motor directions
			switch(this->current_movement_state)
			{
				//IDLE: Robot is not moving
				case IDLE:
				{
					//Set all to forward by default
					this->backright_motor_dir 	= NONE;
					this->backleft_motor_dir 	= NONE;
					this->frontright_motor_dir 	= NONE;
					this->frontleft_motor_dir 	= NONE;
					break;
				}

				case FULL_FORWARD:
				{
					//Set all to forward by default
					this->backright_motor_dir 	= FORWARD;
					this->backleft_motor_dir 	= FORWARD;
					this->frontright_motor_dir 	= FORWARD;
					this->frontleft_motor_dir 	= FORWARD;
					break;
				}

				case FULL_REVERSE:
				{
					//Set all to forward by default
					this->backright_motor_dir 	= REVERSE;
					this->backleft_motor_dir 	= REVERSE;
					this->frontright_motor_dir 	= REVERSE;
					this->frontleft_motor_dir 	= REVERSE;
					break;
				}

				case TANK_ROTATE_LEFT:
				{
					//Set all to forward by default
					this->backright_motor_dir 	= FORWARD;
					this->backleft_motor_dir 	= REVERSE;
					this->frontright_motor_dir 	= FORWARD;
					this->frontleft_motor_dir 	= REVERSE;
					break;
				}

				case TANK_ROTATE_RIGHT:
				{
					//Set all to forward by default
					this->backright_motor_dir 	= REVERSE;
					this->backleft_motor_dir 	= FORWARD;
					this->frontright_motor_dir 	= REVERSE;
					this->frontleft_motor_dir 	= FORWARD;
					break;
				}

				case SOFT_ROTATE_LEFT:
				{
					//Set all to forward by default
					this->backright_motor_dir 	= FORWARD;
					this->backleft_motor_dir 	= NONE;
					this->frontright_motor_dir 	= FORWARD;
					this->frontleft_motor_dir 	= NONE;
					break;
				}


				case SOFT_ROTATE_RIGHT:
				{
					//Set all to forward by default
					this->backright_motor_dir 	= NONE;
					this->backleft_motor_dir 	= FORWARD;
					this->frontright_motor_dir 	= NONE;
					this->frontleft_motor_dir 	= FORWARD;
					break;
				}

				//Default case. Shouldn't be a problem.
				default: break;
			}
			//Update the latchValueToTransmit based on our updated movement control state
			UpdateMotorEnableDirectionInLatchValueToTransmit(FRONT_LEFT, this->frontleft_motor_dir);
			UpdateMotorEnableDirectionInLatchValueToTransmit(FRONT_RIGHT, this->frontright_motor_dir);
			UpdateMotorEnableDirectionInLatchValueToTransmit(BACK_LEFT, this->backleft_motor_dir);
			UpdateMotorEnableDirectionInLatchValueToTransmit(BACK_RIGHT, this->backright_motor_dir);
		}

		//Private member variables and functions
		private:

		void UpdateMotorEnableDirectionInLatchValueToTransmit(MOTOR_ID motor, MOTOR_DIRECTION direction)
		{
			//CAUTION! - to move forward the motors on the left and right of the chassis need 'reversed' polarity / direction to move forward or back. Think about what happens when you take a motor on the left and flip it 180. If you set all motors to same direction the robot will rotate around
			//its center
			if(motor == FRONT_RIGHT)
			{
				//Look at the direction
				if(direction == NONE)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_FR_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_FR_B;
				}
				else if(direction == FORWARD)
				{
					this->updatedLatchValueToTransmit |=  MOTOR_FR_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_FR_B;
				}
				else if(direction == REVERSE)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_FR_A;
					this->updatedLatchValueToTransmit |=  MOTOR_FR_B;
				}
			}

			if(motor == BACK_RIGHT)
			{
				//Look at the direction
				if(direction == NONE)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_BR_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_BR_B;
				}
				else if(direction == FORWARD)
				{
					this->updatedLatchValueToTransmit |=  MOTOR_BR_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_BR_B;
				}
				else if(direction == REVERSE)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_BR_A;
					this->updatedLatchValueToTransmit |=  MOTOR_BR_B;
				}
			}
//DIFFERENT SIDES OF CHASSIS -------------------------------------------------------------------------------
			if(motor == FRONT_LEFT)
			{
				//Look at the direction
				if(direction == NONE)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_FL_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_FL_B;
				}
				else if(direction == FORWARD)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_FL_A;
					this->updatedLatchValueToTransmit |=  MOTOR_FL_B;
				}
				else if(direction == REVERSE)
				{
					this->updatedLatchValueToTransmit |=  MOTOR_FL_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_FL_B;
				}
			}
			if(motor == BACK_LEFT)
			{
				//Look at the direction
				if(direction == NONE)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_BL_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_BL_B;
				}
				else if(direction == FORWARD)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_BL_A;
					this->updatedLatchValueToTransmit |=  MOTOR_BL_B;
				}
				else if(direction == REVERSE)
				{
					this->updatedLatchValueToTransmit |=  MOTOR_BL_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_BL_B;
				}
			}
		}

		//Motor directions
		MOTOR_DIRECTION frontleft_motor_dir, frontright_motor_dir, backleft_motor_dir, backright_motor_dir;
		ROBOT_MOVEMENT_STATE current_movement_state;
		uint8_t updatedLatchValueToTransmit;
	};

}

#endif /* SRC_MOTORCONTROL_HPP_ */
