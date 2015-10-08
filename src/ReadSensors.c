/*
 * ReadSensors.c
 *
 *  Created on: 2015. okt. 8.
 *      Author: Csabi
 */

#include "ReadSensors.h"

void ReadSensors()
{

}

void EnableDrivers()
{

}
void DisableDrivers()
{

}
void SetLeds(uint16_t pattern)
{

}
void ShiftLeds(uint8_t amount)
{

}

void EnableMUX()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET); // negált az EN jel
}
void DisableMUX()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
}
void SetMUX(uint8_t num)
{

}

uint16_t ReadADC()
{
	return 1;
}

















