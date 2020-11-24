#ifndef SHIFT_REGISTER_UPDATE_STATE_MACHINE
#define SHIFT_REGISTER_UPDATE_STATE_MACHINE

#include "MotorControl.hpp"

namespace ECE477_17
{
	enum SHIFT_REG_STATES{STANDBY, LATCH, CLK_RST, TRANSFER_BIT, CLK_SET, UNLATCH};

	struct ShiftRegisterUpdateStateMachine
	{
		//Constructor
		ShiftRegisterUpdateStateMachine(void)
		{
			this->currentBitToSend 	= 0;
			this->beginTransmit 	= false;
			this->current_state 		= STANDBY;
		}

		bool IsCurrentlyUpdatingShiftRegister(void) const
		{
			return this->current_state != STANDBY;
		}

		bool ReadyToUpdateShiftRegister(void) const
		{
			return this->current_state == STANDBY;
		}

		//Update FSM
		void UpdateState(uint8_t latchValueToTransmit)
		{
			//Observe current state
			switch(this->current_state)
			{
				//Standby state
				case STANDBY:
				{
					//Start transmitting? Check it
					if(this->beginTransmit == true)
					{
						this->current_state = LATCH;
						//Reset beginTransmit
						this->beginTransmit = false;
					}
					else
					{
						this->current_state = STANDBY;
					}
				}break;

				case LATCH:
				{
					//Set LATCH low
					SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_LATCH;
					//Go to CLK_RST
					this->current_state = CLK_RST;
				}break;

				case CLK_RST:
				{
					//Set CLK low
					SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_CLK;
					//Transfer a bit next!
					this->current_state = TRANSFER_BIT;
				}break;

				case TRANSFER_BIT:
				{
					//Get bit to transfer
					uint8_t ithBitValueOfLatchValueToTransmit = latchValueToTransmit & _BS((7-(this->currentBitToSend)));
					//Set HI or LOW depending on the bit value at index 'i'
					if (ithBitValueOfLatchValueToTransmit)
					{
						SHIFT_REGISTER_GPIO->ODR |=  SHIFT_REGISTER_DATA;
					}
					else
					{
						SHIFT_REGISTER_GPIO->ODR &= ~SHIFT_REGISTER_DATA;
					}

					//LED go brrrt
					GPIOD->ODR ^= _BS(13); //Turn on orange LED

					//Increment bit value
					this->currentBitToSend++;

					//Go to CLK_SET
					this->current_state = CLK_SET;
				}break;

				case CLK_SET:
				{
					//Set CLK high.
					SHIFT_REGISTER_GPIO->ODR |= SHIFT_REGISTER_CLK;
					//check currentBitToSend
					//This should be 8 because the currentBitToSend will be 7 (last bit) before incrementing in TRANSFER_BIT. Therefore it will be 8 when arriving to this state.
					if(currentBitToSend >= 8)
					{
						//Reset currentBitToSend, and go to UNLATCH
						currentBitToSend = 0;
						this->current_state = UNLATCH;
					}
					//We have more bits to send. Loop back to CLK_RST
					else
					{
						this->current_state = CLK_RST;
					}
				}break;

				case UNLATCH:
				{
					//LED go brrrt
					GPIOD->ODR &= ~_BS(13); //Turn on orange LED

					//Set LATCH high
					SHIFT_REGISTER_GPIO->ODR |= SHIFT_REGISTER_LATCH;
					//Go to standby next
					this->current_state = STANDBY;
				}break;

				default: break;
			}
		}

		//State machine variables
		int currentBitToSend;
		bool beginTransmit;

		//Current state enum
		SHIFT_REG_STATES current_state;
	};

}

#endif
