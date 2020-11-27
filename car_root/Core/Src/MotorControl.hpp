/*
 * MotorControl.h
 *
 *  Created on: Oct 5, 2020
 *      Author: gorshborsh
 */

#ifndef SRC_MOTORCONTROL_HPP_
#define SRC_MOTORCONTROL_HPP_

#include "stm32f4xx.h"

//Bitshift
#define _BS(x) (1 << x)

namespace ECE477_17
{
	//Enum that describes whether a particular motor should drive forwards or backwards
	enum MOTOR_DIRECTION{ FORWARD, REVERSE, NONE };
	//Enum that describes how the robot should move. This determines the MOTOR_DIRECTION of each motor
	enum ROBOT_MOVEMENT_STATE{ IDLE, FULL_FORWARD, FULL_REVERSE, TANK_ROTATE_LEFT, TANK_ROTATE_RIGHT };
	enum MOTOR_ID {LEFT_SIDE, RIGHT_SIDE};

	volatile uint32_t* const LEFT_SIDE_MOTOR_EN_A 	= &(TIM1->CCR1);
	volatile uint32_t* const LEFT_SIDE_MOTOR_EN_B 	= &(TIM1->CCR2);
	volatile uint32_t* const RIGHT_SIDE_MOTOR_EN_A 	= &(TIM1->CCR3);
	volatile uint32_t* const RIGHT_SIDE_MOTOR_EN_B 	= &(TIM1->CCR4);

	const uint32_t PWM_DUTY_CYCLE_FULL_SPEED = TIM1->ARR;
	const uint32_t PWM_DUTY_CYCLE_LOW_SPEED  = (uint32_t)( (TIM1->ARR) * 0.75f);

	//Struct for controlling motors
	struct RobotMovementController
	{
		//Constructor
		RobotMovementController(void)
		{
			//Set all to forward by default
			this->right_side_direction 	= NONE;
			this->left_side_direction 	= NONE;
			//Set movement state to Idle
			this->current_movement_state = IDLE;
			//Set speed - full speed by default
			this->speed = PWM_DUTY_CYCLE_FULL_SPEED;
		}

		//Destructor (unused)
		~RobotMovementController(void){}

		//Helper functions to set speed
		void SetHighSpeed(void) { this->speed = PWM_DUTY_CYCLE_FULL_SPEED; }
		void SetLowSpeed(void) { this->speed = PWM_DUTY_CYCLE_LOW_SPEED; }

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
					this->right_side_direction 	= NONE;
					this->left_side_direction 	= NONE;
				}break;

				case FULL_FORWARD:
				{
					this->right_side_direction 	= FORWARD;
					this->left_side_direction 	= FORWARD;
				}break;

				case FULL_REVERSE:
				{
					this->right_side_direction 	= REVERSE;
					this->left_side_direction 	= REVERSE;
				}break;

				case TANK_ROTATE_LEFT:
				{
					this->right_side_direction 	= FORWARD;
					this->left_side_direction 	= REVERSE;
				}break;

				case TANK_ROTATE_RIGHT:
				{
					this->right_side_direction 	= REVERSE;
					this->left_side_direction 	= FORWARD;
				}break;
				//Default case. Shouldn't be a problem.
				default: break;
			}
			//Update PWM CCR registers
			UpdatePWMCCRxRegisterBasedOnDirection(this->left_side_direction, LEFT_SIDE_MOTOR_EN_A, LEFT_SIDE_MOTOR_EN_B);
			UpdatePWMCCRxRegisterBasedOnDirection(this->right_side_direction, RIGHT_SIDE_MOTOR_EN_A, RIGHT_SIDE_MOTOR_EN_B);
		}

		//Private member variables and functions
		private:

		//Helper function to set
		void UpdatePWMCCRxRegisterBasedOnDirection(MOTOR_DIRECTION dir, volatile uint32_t* const CCRxPtrA, volatile uint32_t* const CCRxPtrB )
		{
			if(dir == NONE)
			{
				(*CCRxPtrA) = 0;
				(*CCRxPtrB) = 0;
			}
			else if(dir == FORWARD)
			{
				(*CCRxPtrA) = this->speed;
				(*CCRxPtrB) = 0;
			}
			else if(dir == REVERSE)
			{
				(*CCRxPtrA) = 0;
				(*CCRxPtrB) = this->speed;
			}
		}

		//Speed to set PWM duty cycles
		uint32_t speed;

		//Motor directions
		MOTOR_DIRECTION left_side_direction, right_side_direction;
		ROBOT_MOVEMENT_STATE current_movement_state;
	};

}

#endif /* SRC_MOTORCONTROL_HPP_ */
