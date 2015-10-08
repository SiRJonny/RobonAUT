/*
 * ReadSensors.h
 *
 *  Created on: 2015. okt. 8.
 *      Author: Csabi
 */

#ifndef INCLUDE_READSENSORS_H_
#define INCLUDE_READSENSORS_H_


void ReadSensors();
void EnableDrivers();
void DisableDrivers();
void SetLeds(uint16_t pattern);
void ShiftLeds(uint8_t);

void EnableMUX();
void DisableMUX();
void SetMUX(uint8_t);

uint16_t ReadADC();

#endif /* INCLUDE_READSENSORS_H_ */
