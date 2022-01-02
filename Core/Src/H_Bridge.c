#include "TB6612.h"


void TB6612_SetDirection( uint8_t dir, uint8_t dirPort, uint8_t *lastState)
{

	switch (dirPort)
	{
		case TB6612_DIR_PORT_A:

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

		case TB6612_DIR_PORT_B:

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

void TB6612_ToggleDirection( uint8_t dirPort, uint8_t *lastState)
{


		switch (dirPort)
		{
			case TB6612_DIR_PORT_A:

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


			case TB6612_DIR_PORT_B:

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
