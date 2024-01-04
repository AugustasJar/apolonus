/*
 * encoder.c
 *
 *  Created on: Apr 12, 2023
 *      Author: augus
 */


#include "encoder.h"

void encoder_init (encoder* Encoder,GPIO_TypeDef *GPIOAx, uint16_t pinA,GPIO_TypeDef *GPIOBx, uint16_t pinB, double encoder_step){

	Encoder->GPIOAx = GPIOAx;
	Encoder->GPIOBx = GPIOBx;

	Encoder->pinA = pinA;
	Encoder->pinB = pinB;
	Encoder->encoder_step = encoder_step;
}
void encoder_update(encoder *Encoder){

			Encoder->ref_signal_A = HAL_GPIO_ReadPin(Encoder->GPIOAx,Encoder->pinA);
			Encoder->ref_signal_B = HAL_GPIO_ReadPin(Encoder->GPIOBx,Encoder->pinB);
			Encoder->last_pos = Encoder->pos;
	    	if (Encoder->ref_signal_B >0)
	    	{
	    		Encoder->raw_pos++;
	    		Encoder->pos = Encoder->pos + Encoder->encoder_step;
	    	}
	    	else
	    	{
	    		Encoder->raw_pos--;
				Encoder->pos = Encoder->pos - Encoder->encoder_step;
	    	}
	    }

