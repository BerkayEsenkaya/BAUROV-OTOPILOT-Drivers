/*
 * PressureSensor.c
 *
 *  Created on: Aug 13, 2025
 *      Author: Berkay Esenkaya
 */

#include <stdint.h>
#include "PressureSensor.h"
#include "BAR30.h"
#include "Filters.h"
#include "main.h"

PressureSensor_TypeDef_T PressureSensor_1;

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void PressureSensor_Init(PressureSensor_TypeDef_T *handle, uint8_t PressureSensorNo, uint8_t I2C_No, uint8_t DevI2C_Address){
	BAR30_Init(PressureSensorNo, I2C_No, DevI2C_Address);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void PressureSensor_Execute(PressureSensor_TypeDef_T *handle, uint8_t PressureSensorNo){

	BAR30_Execute(PressureSensorNo);

	PressureSensor_PopulateAllData(handle);

	MED_Filter_u32(&MED_PressureSensor_1, handle->PressureData);

	AVG_Filter_u32(&AVG_PressureSensor_1, MED_PressureSensor_1.MED_FilteredData);

	handle->FilteredPressureDataPascal = AVG_PressureSensor_1.AVG_FilteredData;
}

void PressureSensor_PopulateAllData(PressureSensor_TypeDef_T *handle){
	handle->PressureData = BAR30_1.Pascal;
	handle->TempData = BAR30_1.miliCentigrad;
}
