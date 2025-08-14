/*
 * BAR30.c
 *
 *  Created on: Aug 13, 2025
 *      Author: Berkay Esenkaya
 */
#include <stdint.h>
#include <math.h>

#include "main.h"
#include "BAR30.h"
#include "BAR30_CommPorter.h"

BAR30_Sensor_T BAR30_1 ,BAR30_2, BAR30_3;
uint32_t tickpre, tickpost, bar30startpressure_tick, readpressure_start_temp_tick, readttemp_and_convert_tick;
/**********************************************************
 * GLOBAL FUNCTIONS
 *********************************************************/
/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_Init(uint8_t BAR30_No, uint8_t i2cNo, uint8_t devI2CAddress){
	BAR30_ReturnTypeDef_T res;
	BAR30_Sensor_T *bar;
	uint8_t i=0;

	bar = BAR30_GetHandle(BAR30_No);

	bar->devParam.I2C_No = i2cNo;
	bar->devParam.DevAdress = devI2CAddress;

	res = BAR30_Reset(bar);
	HAL_Delay(25);

	while (BAR30_Get_PromValues(bar) == BAR30_ERROR){
		if(i++ > 10)
			break;
	}

	return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void BAR30_Execute(uint8_t BAR30_No){
	BAR30_Sensor_T *bar;
    static int8_t i=0;

	bar = BAR30_GetHandle(BAR30_No);
	if(i==0)
		BAR30_StartPressureConversionWithOSR(bar, BAR30_COMMANDS_D1_OSR_2048);

	if(i==1){
		BAR30_ReadPressure(bar);
		BAR30_StartTempConversionWithOSR(bar, BAR30_COMMANDS_D2_OSR_1024);
	}

	if(i==2){
		BAR30_ReadTemp(bar);
		BAR30_ConvertCentigrade(bar);
		BAR30_ConvertPascal(bar);
		i= -1;
	}

	i++;


}

/**********************************************************
 * PRIVATE FUNCTIONS
 *********************************************************/
/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_Reset(BAR30_Sensor_T *handle){
	return BAR30_SendReceive(handle, BAR30_COMMANDS_RESET, 0, 0);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_StartPressureConversionWithOSR(BAR30_Sensor_T *handle, uint8_t osr){
	return BAR30_SendReceive(handle, osr, 0, 0);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_StartTempConversionWithOSR(BAR30_Sensor_T *handle, uint8_t osr){
	return BAR30_SendReceive(handle, osr, 0, 0);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_ReadPressure(BAR30_Sensor_T *handle){
	uint8_t RxBuff[3];

	if(!BAR30_SendReceive(handle, BAR30_COMMANDS_ADC_READ, RxBuff, 3))
		handle->Pressure_RawData = (uint32_t)(RxBuff[2] | (RxBuff[1]<<8) | (RxBuff[0]<<16));

	else
		return BAR30_ERROR;
	return BAR30_OK;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_ReadTemp(BAR30_Sensor_T *handle){
	uint8_t RxBuff[3];

	if(!BAR30_SendReceive(handle, BAR30_COMMANDS_ADC_READ, RxBuff, 3))
		handle->Temperature_RawData = (uint32_t)(RxBuff[2] | (RxBuff[1]<<8) | (RxBuff[0]<<16));

	else
		return BAR30_ERROR;
	return BAR30_OK;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_Get_PromValues(BAR30_Sensor_T *handle){
	uint8_t RxBuff[2], crc = 0;

	for(int i=0; i<7;i++){
		if(!BAR30_SendReceive(handle, BAR30_COMMANDS_PROM_READ_1 + i*2, RxBuff, 2))
			handle->PromParameters.PromData[i]= RxBuff[1] | RxBuff[0]<<8;
		else
			return BAR30_ERROR;
	}

	crc = handle->PromParameters.PromData[0]>>12;

	if(BAR30_CalculateCRC_4(handle->PromParameters.PromData) == crc)
		return BAR30_OK;
	else
		return BAR30_ERROR;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_Sensor_T* BAR30_GetHandle(uint8_t Bar30_No){
	switch(Bar30_No){
		case 1  : return &BAR30_1;
		case 2  : return &BAR30_2;
		case 3  : return &BAR30_3;
		default : return &BAR30_1;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
uint8_t BAR30_CalculateCRC_4(uint16_t *n_prom) // n_prom defined as 8x unsigned int (n_prom[8])
{
	int cnt; // simple counter
	unsigned int n_rem=0; // crc remainder
	unsigned char n_bit;
	n_prom[0]=((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
	n_prom[7]=0; // Subsidiary value, set to 0

	for (cnt = 0; cnt < 16; cnt++){ // choose LSB or MSB
		if (cnt%2==1)
			n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else
			n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);

		for (n_bit = 8; n_bit > 0; n_bit--){
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}

	n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
	return (n_rem ^ 0x00);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void BAR30_ConvertPascal(BAR30_Sensor_T *handle){
	handle->PromParameters.Off = handle->PromParameters.PromData[2]*pow(2, 16) + (handle->PromParameters.PromData[4]*handle->PromParameters.dT)/pow(2, 7);
	handle->PromParameters.Sens = handle->PromParameters.PromData[1]*pow(2, 15) + (handle->PromParameters.PromData[3]*handle->PromParameters.dT)/pow(2, 8);
	handle->PromParameters.P = (handle->Pressure_RawData*handle->PromParameters.Sens/pow(2,21) - handle->PromParameters.Off)/pow(2,23);
	handle->Pascal = handle->PromParameters.P*10000;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void BAR30_ConvertCentigrade(BAR30_Sensor_T *handle){
	handle->PromParameters.dT = handle->Temperature_RawData-(handle->PromParameters.PromData[5]*pow(2,15));
	handle->PromParameters.Temp = 2000 + (handle->PromParameters.dT*handle->PromParameters.PromData[6])/pow(2,23);
	handle->miliCentigrad = handle->PromParameters.Temp*10;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BAR30_ReturnTypeDef_T BAR30_SendReceive(BAR30_Sensor_T *handle, uint8_t Command, uint8_t *rxBuff, uint8_t rxLenght){
	uint8_t txBuff[1], txLenght;
	txBuff[0] = Command;
	txLenght = 1;
	return BAR30_CommPorter_SendReceive(handle->devParam.I2C_No, handle->devParam.DevAdress, txBuff, txLenght, rxBuff, rxLenght);
}
