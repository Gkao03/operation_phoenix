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
	const auto SHIFT_REGISTER_LATCH 		= GPIO_PIN_1;
	const auto SHIFT_REGISTER_DATA			= GPIO_PIN_2;
	const auto SHIFT_REGISTER_CLK			= GPIO_PIN_3;
	const auto SHIFT_REGISTER_ENABLE		= GPIO_PIN_4;
	//Values corresponding to MOTOR_X ENABLE and MOTOR_X DIRECTION for 'updatedLatchValueToTransmit'
	const uint8_t MOTOR_FL_A					= 0x01; //0000 0000
	const uint8_t MOTOR_FL_B					= 0x02; //0000 0010
	const uint8_t MOTOR_BL_A					= 0x04; //0000 0100
	const uint8_t MOTOR_BL_B					= 0x08; //0000 1000
	const uint8_t MOTOR_FR_A					= 0x10; //0001 0000
	const uint8_t MOTOR_FR_B					= 0x20; //0010 0000
	const uint8_t MOTOR_BR_A					= 0x40; //0100 0000
	const uint8_t MOTOR_BR_B					= 0x80; //1000 0000

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

		void SetLatchPinsToDefaultState(void)
		{
			//Set MOTORLATCH high. We are no longer transmitting data
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_LATCH, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_DATA, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_CLK, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_ENABLE, GPIO_PIN_RESET);

			ShiftRegisterAssignMotorEnableDirectionValues();
		}

		//Assign new motor values to shift register
		void ShiftRegisterAssignMotorEnableDirectionValues_TIM3_Interrupt(int i) const
		{
			//HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_ENABLE, GPIO_PIN_SET);
			//Set LATCH to LOW
			//HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_LATCH, GPIO_PIN_RESET);

			//Set MOTORDATA LOW
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_DATA, GPIO_PIN_RESET);

			//Set MOTORCLK to LOW
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_CLK, GPIO_PIN_RESET);

			//Get the i'th bit position of latchValue
			uint8_t ithBitValueOfLatchValueToTransmit = (this->updatedLatchValueToTransmit >> i) & 0x01;

			//Transmit the bits of the latchValueToTransmit
			if (ithBitValueOfLatchValueToTransmit)
			{
				HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_DATA, GPIO_PIN_SET);
			}

			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_CLK, GPIO_PIN_SET);

			//Set MOTORLATCH high. We are no longer transmitting data
			//HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_LATCH, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_ENABLE, GPIO_PIN_RESET);
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


			ShiftRegisterAssignMotorEnableDirectionValues();
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
					this->updatedLatchValueToTransmit &= ~MOTOR_FR_A || ~MOTOR_FR_B;
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
					this->updatedLatchValueToTransmit &= ~MOTOR_BR_A || ~MOTOR_BR_B;
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
					this->updatedLatchValueToTransmit &= ~MOTOR_FL_A || ~MOTOR_FL_B;
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
					this->updatedLatchValueToTransmit &= ~MOTOR_BL_A || ~MOTOR_BL_B;
				}
				else if(direction == FORWARD)
				{
					this->updatedLatchValueToTransmit |=  MOTOR_BL_A;
					this->updatedLatchValueToTransmit &= ~MOTOR_BL_B;
				}
				else if(direction == REVERSE)
				{
					this->updatedLatchValueToTransmit &= ~MOTOR_BL_A;
					this->updatedLatchValueToTransmit |=  MOTOR_BL_B;
				}
			}
		}

		//Assign new motor values to shift register
		void ShiftRegisterAssignMotorEnableDirectionValues(void) const
		{
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_ENABLE, GPIO_PIN_SET);
			//Set LATCH to LOW
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_LATCH, GPIO_PIN_RESET);

			//Set MOTORDATA LOW
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_DATA, GPIO_PIN_RESET);

			for (int i = 0; i < 8; i++)
			{

				//Set MOTORCLK to LOW
				HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_CLK, GPIO_PIN_RESET);

				//Get the i'th bit position of latchValue
				uint8_t ithBitValueOfLatchValueToTransmit = (this->updatedLatchValueToTransmit >> i) & 0x01;

				//Transmit the bits of the latchValueToTransmit
			    if (ithBitValueOfLatchValueToTransmit)
			    {
			    	HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_DATA, GPIO_PIN_SET);
			    }
			    else
			    {
			    	HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_DATA, GPIO_PIN_RESET);
			    }

			    HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_CLK, GPIO_PIN_SET);

			}

			//Set MOTORLATCH high. We are no longer transmitting data
			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_LATCH, GPIO_PIN_SET);

			HAL_GPIO_WritePin(SHIFT_REGISTER_GPIO, SHIFT_REGISTER_ENABLE, GPIO_PIN_RESET);
		}


		//Motor directions
		MOTOR_DIRECTION frontleft_motor_dir, frontright_motor_dir, backleft_motor_dir, backright_motor_dir;
		ROBOT_MOVEMENT_STATE current_movement_state;
		uint8_t updatedLatchValueToTransmit;

	};

}

#endif /* SRC_MOTORCONTROL_HPP_ */
