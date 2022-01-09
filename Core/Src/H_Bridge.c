#include "H_Bridge.h"


void H_Bridge_SetDirection( uint8_t dir, uint8_t dirPort, Direction *lastState)
{

	switch (dirPort)
	{
		case H_BRIDGE_DIR_PORT_A:

					switch (dir)
					{

					case DIR_CW:


						HAL_GPIO_WritePin(A_OUT1_GPIO_Port, A_OUT1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(A_OUT2_GPIO_Port, A_OUT2_Pin, GPIO_PIN_RESET);

						*lastState = DIR_CW;

					break;

					case DIR_CCW:

						HAL_GPIO_WritePin(A_OUT2_GPIO_Port, A_OUT2_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(A_OUT1_GPIO_Port, A_OUT1_Pin, GPIO_PIN_RESET);

						*lastState = DIR_CCW;

					break;

					default:
					break;

					}
			break;

		case H_BRIDGE_DIR_PORT_B:

					switch (dir)
					{

					case DIR_CW:


						HAL_GPIO_WritePin(B_OUT1_GPIO_Port, B_OUT1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(B_OUT2_GPIO_Port, B_OUT2_Pin, GPIO_PIN_RESET);

						*lastState = DIR_CW;

					break;

					case DIR_CCW:


						HAL_GPIO_WritePin(B_OUT2_GPIO_Port, B_OUT2_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(B_OUT1_GPIO_Port, B_OUT1_Pin, GPIO_PIN_RESET);

						*lastState = DIR_CCW;

					break;

					default:
					break;

				   }
		break;
	}
}

void H_Bridge_ToggleDirection( uint8_t dirPort, Direction *lastState)
{


		switch (dirPort)
		{
			case H_BRIDGE_DIR_PORT_A:

						switch (*lastState)
						{

						case DIR_CCW:


							HAL_GPIO_WritePin(A_OUT1_GPIO_Port, A_OUT1_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(A_OUT2_GPIO_Port, A_OUT2_Pin, GPIO_PIN_RESET);

							*lastState = DIR_CW;

						break;

						case DIR_CW:


							HAL_GPIO_WritePin(A_OUT2_GPIO_Port, A_OUT2_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(A_OUT1_GPIO_Port, A_OUT1_Pin, GPIO_PIN_RESET);

							*lastState = DIR_CCW;

						break;

						}
			break;


			case H_BRIDGE_DIR_PORT_B:

						switch (*lastState)
						{

						case DIR_CCW:


							HAL_GPIO_WritePin(B_OUT1_GPIO_Port, B_OUT1_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(B_OUT2_GPIO_Port, B_OUT2_Pin, GPIO_PIN_RESET);

							*lastState = DIR_CW;

						break;

						case DIR_CW:


							HAL_GPIO_WritePin(B_OUT2_GPIO_Port, B_OUT2_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(B_OUT1_GPIO_Port, B_OUT1_Pin, GPIO_PIN_RESET);

							*lastState = DIR_CCW;

						break;

					   }
			break;

		}
}
