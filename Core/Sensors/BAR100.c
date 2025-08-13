/*
 * BAR100.c
 *
 *  Created on: Jul 21, 2025
 *      Author: Berkay Esenkaya
 */


/**
 * \file BAR30.c
 * \author Berkay Esenkaya (BAUROV Software Team)
 * \brief BAR30 sensor control functions
 * \date 24.02.2025
 *
 */

#include <stdint.h>
#include "BAR100.h"
#include "BAR100_CommPorter.h"
#include "main.h"


BAR100_Sensor_T BAR100_1;

float Convert_IEE754_To_FloatValue(const uint32_t value){
	float f;
	memcpy(&f, &value, sizeof(float));
	return f;
}

BAR100_ReturnTypeDef_T BAR100_Calc_Pressure(BAR100_Sensor_T *handle){
	switch(handle->mode){
		case Zero_At_Atm    : handle->miliPressure = (((handle->Pressure_RawData - handle->PMin)*((handle->PMax-handle->PMin)*1000))/handle->PMax)-handle->PMin; break;
		case Zero_At_1Bar   : handle->miliPressure = (((handle->Pressure_RawData-16384)*((handle->PMax-handle->PMin)*1000))/32768)+handle->PMin;break;
		case Zero_At_Vacuum : handle->miliPressure = ((handle->Pressure_RawData-handle->PMin)*((handle->PMax-handle->PMin)*1000))/handle->PMax+handle->PMin;     break;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR100_ReturnTypeDef_T BAR100_Init(BAR100_Sensor_T *handle, uint8_t i2cNo, uint8_t devI2CAddress){
	handle->devParam.I2C_No = i2cNo;
	handle->devParam.DevAdress = devI2CAddress;
	return BAR100_OK;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR100_ReturnTypeDef_T BAR100_StartPressureConversion(BAR100_Sensor_T *handle){
	BAR100_ReturnTypeDef_T res;
	res =  BAR100_Write(handle, 0xA0);
	return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR100_ReturnTypeDef_T BAR100_ReadPressure(BAR100_Sensor_T *handle){
	uint8_t RxBuff[5], state;
	if(!BAR100_Read(handle, 0xAC, RxBuff)){
		handle->Pressure_RawData = RxBuff[2] | (RxBuff[1]<<8);
	    handle->Temperature_RawData = RxBuff[4] | (RxBuff[3]<<8);
	    state = RxBuff[0];
	    BAR100_Calc_Pressure(handle);
	}
	else
		return BAR100_ERROR;
}

BAR100_ReturnTypeDef_T BAR100_Get_ScalingValues(BAR100_Sensor_T *handle){
	uint8_t RxBuff[3];
	uint16_t ScaleData[5];
    uint32_t iee754;
	for(int i=0; i<5;i++){
		if(!BAR100_Read(handle, BAR100_MTP_ADDRESS_SCALING_BASE + i, RxBuff)){
			ScaleData[i]= RxBuff[2]<<0 | RxBuff[1]<<8;
		}else{
			return BAR100_ERROR;
		}
	}

    handle->mode = ScaleData[0]&(0x0003);
    iee754 = ScaleData[1]<<16 | ScaleData[2];
    handle->PMin = Convert_IEE754_To_FloatValue(iee754);
    iee754 = ScaleData[3]<<16 | ScaleData[4];
    handle->PMax = Convert_IEE754_To_FloatValue(iee754);
	return BAR100_OK;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR100_ReturnTypeDef_T BAR100_Write(BAR100_Sensor_T *handle, uint8_t command){
	uint8_t TxBuff[1];
	TxBuff[0] = command;
	return BAR100_SendReceive(handle, TxBuff, 1, NULL, 0);
}

BAR100_ReturnTypeDef_T BAR100_Read(BAR100_Sensor_T *handle, uint8_t command, uint8_t *buffer){
	uint8_t lenght;

	if(command == 0xAC)
		lenght = 5;
	else
		lenght = 3;

	return BAR100_SendReceive(handle, &command, 1, buffer, lenght);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR100_ReturnTypeDef_T BAR100_SendReceive(BAR100_Sensor_T *handle, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght){
	HAL_Delay(10);
	return BAR100_CommPorter_SendReceive(handle->devParam.I2C_No, handle->devParam.DevAdress, txBuff, txLenght, rxBuff, rxLenght);
}
