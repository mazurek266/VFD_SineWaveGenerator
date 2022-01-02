/*
 * TB6612.h
 *
 *  Created on: 13 gru 2021
 *      Author: reks
 */

#ifndef INC_TB6612_H_
#define INC_TB6612_H_

#include <main.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef enum
{
	DIR_CW,
	DIR_CCW
}Direction;

#define TB6612_DIR_PORT_A 0
#define TB6612_DIR_PORT_B 1

#define CHANNEL_A_INIT_DIR DIR_CW
#define CHANNEL_B_INIT_DIR DIR_CW

void TB6612_SetDirection( uint8_t dir, uint8_t dirPort, uint8_t *lastState);
void TB6612_ToggleDirection( uint8_t dirPort, uint8_t *lastState);
#endif /* INC_TB6612_H_ */