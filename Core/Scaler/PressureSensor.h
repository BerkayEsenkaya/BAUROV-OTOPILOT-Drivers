/*
 * PressureSensor.h
 *
 *  Created on: Aug 13, 2025
 *      Author: Berkay Esenkaya
 */

#ifndef SCALER_PRESSURESENSOR_H_
#define SCALER_PRESSURESENSOR_H_

typedef struct{
	uint32_t PressureData;//pascal
	uint32_t TempData;
	uint32_t FilteredPressureDataPascal;
}PressureSensor_TypeDef_T;

void PressureSensor_Init(PressureSensor_TypeDef_T *handle, uint8_t PressureSensorNo, uint8_t I2C_No, uint8_t DevI2C_Address);
void PressureSensor_Execute(PressureSensor_TypeDef_T *handle, uint8_t PressureSensorNo);
void PressureSensor_PopulateAllData(PressureSensor_TypeDef_T *handle);

#endif

