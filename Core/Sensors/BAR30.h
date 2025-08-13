/*
 * BAR30.h
 *
 *  Created on: Aug 13, 2025
 *      Author: Berkay Esenkaya
 */

#ifndef SENSORS_BAR30_H_
#define SENSORS_BAR30_H_

#define BAR30_I2C_ADDRESS 0x76

#define BAR30_COMMANDS_ADC_READ     0x00
#define BAR30_COMMANDS_RESET        0x1E
#define BAR30_COMMANDS_D1_OSR_256   0x40
#define BAR30_COMMANDS_D1_OSR_512   0x42
#define BAR30_COMMANDS_D1_OSR_1024  0x44
#define BAR30_COMMANDS_D1_OSR_2048  0x46
#define BAR30_COMMANDS_D1_OSR_4096  0x48
#define BAR30_COMMANDS_D1_OSR_8192  0x4A
#define BAR30_COMMANDS_D2_OSR_256   0x50
#define BAR30_COMMANDS_D2_OSR_512   0x52
#define BAR30_COMMANDS_D2_OSR_1024  0x54
#define BAR30_COMMANDS_D2_OSR_2048  0x56
#define BAR30_COMMANDS_D2_OSR_4096  0x58
#define BAR30_COMMANDS_D2_OSR_8192  0x5A
#define BAR30_COMMANDS_PROM_READ_1  0xA0
#define BAR30_COMMANDS_PROM_READ_2  0xA2
#define BAR30_COMMANDS_PROM_READ_3  0xA4
#define BAR30_COMMANDS_PROM_READ_4  0xA6
#define BAR30_COMMANDS_PROM_READ_5  0xA8
#define BAR30_COMMANDS_PROM_READ_6  0xAA
#define BAR30_COMMANDS_PROM_READ_7  0xAC

#define BAR30_TIMEOUT_MS_OSR_256  1
#define BAR30_TIMEOUT_MS_OSR_512  2
#define BAR30_TIMEOUT_MS_OSR_1024 3
#define BAR30_TIMEOUT_MS_OSR_2048 5
#define BAR30_TIMEOUT_MS_OSR_4096 10
#define BAR30_TIMEOUT_MS_OSR_8192 19


#define BAR30_INCORRECTED_DATA 9999

typedef enum{
	BAR30_OK,
	BAR30_ERROR,
}BAR30_ReturnTypeDef_T;

typedef struct{
	uint16_t PromData[8];
	uint32_t dT;
	uint32_t Temp;
	int64_t  Off;
	int64_t  Sens;
	int32_t  P;
}BAR30_PromParameters_T;

typedef struct{
	uint8_t I2C_No;
	uint8_t DevAdress;
}BAR30_DeviceParam_T;

typedef struct{
	BAR30_DeviceParam_T devParam;
	BAR30_PromParameters_T PromParameters;
	uint32_t Temperature_RawData;
	uint32_t Pressure_RawData;
	uint32_t miliCentigrad;
	uint32_t Pascal;
}BAR30_Sensor_T;

extern BAR30_Sensor_T BAR30_1 ,BAR30_2, BAR30_3;

BAR30_ReturnTypeDef_T BAR30_Init(uint8_t BAR30_No, uint8_t i2cNo, uint8_t devI2CAddress);
void BAR30_Execute(uint8_t BAR30_No);
BAR30_ReturnTypeDef_T BAR30_Reset(BAR30_Sensor_T *handle);
BAR30_ReturnTypeDef_T BAR30_StartPressureConversionWithOSR(BAR30_Sensor_T *handle, uint8_t osr);
BAR30_ReturnTypeDef_T BAR30_StartTempConversionWithOSR(BAR30_Sensor_T *handle, uint8_t osr);
BAR30_ReturnTypeDef_T BAR30_ReadPressure(BAR30_Sensor_T *handle);
BAR30_ReturnTypeDef_T BAR30_ReadTemp(BAR30_Sensor_T *handle);
BAR30_ReturnTypeDef_T BAR30_Get_PromValues(BAR30_Sensor_T *handle);
BAR30_ReturnTypeDef_T BAR30_Write(BAR30_Sensor_T *handle, uint8_t command);
BAR30_ReturnTypeDef_T BAR30_Read(BAR30_Sensor_T *handle, uint8_t command, uint8_t *buffer);
BAR30_ReturnTypeDef_T BAR30_SendReceive(BAR30_Sensor_T *handle, uint8_t Command, uint8_t *rxBuff, uint8_t rxLenght);
BAR30_Sensor_T* BAR30_GetHandle(uint8_t Bar30_No);
uint8_t BAR30_CalculateCRC_4(uint16_t *n_prom);
void BAR30_ConvertPascal(BAR30_Sensor_T *handle);
void BAR30_ConvertCentigrade(BAR30_Sensor_T *handle);
#endif /* SENSORS_BAR30_H_ */
