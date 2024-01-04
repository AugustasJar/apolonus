/*
 * encoder.h
 *
 *  Created on: Apr 12, 2023
 *      Author: augus
 */
#include "stm32f1xx_hal.h"

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

typedef struct
{
	uint16_t ref_signal_A;
	uint16_t ref_signal_B;

	uint16_t pinA,pinB;
	GPIO_TypeDef *GPIOAx,*GPIOBx;
	double encoder_step;

	int16_t raw_pos;
	int16_t raw_last_post;

	double pos;
	double last_pos;

	double speed;
} encoder;

void encoder_init (encoder* Encoder,GPIO_TypeDef *GPIOAx, uint16_t pinA,GPIO_TypeDef *GPIOBx, uint16_t pinB, double encoder_step);
void encoder_update(encoder *Encoder);

#endif /* SRC_ENCODER_H_ */
