#ifndef SENSORS_BAR100_H_
#define SENSORS_BAR100_H_


#define BAR100_DEVICE_ADDRESS (0x40)

#define BAR100_MTP_ADDRESS_CUST_ID_0 (0x00)
#define BAR100_MTP_ADDRESS_CUST_ID_1 (0x01)
#define BAR100_MTP_ADDRESS_SCALING_BASE (0x12)
#define BAR100_MTP_ADDRESS_SCALING_1 (0x13)
#define BAR100_MTP_ADDRESS_SCALING_2 (0x14)
#define BAR100_MTP_ADDRESS_SCALING_3 (0x15)
#define BAR100_MTP_ADDRESS_SCALING_4 (0x16)

#define BAR100_COMMAND_START_CONVERSION (0xA0)
#define BAR100_COMMAND_READ_CONVERSION_DATA (0xAC)

#define BAR100_INCORRECTED_DATA 9999

typedef enum{
	BAR100_OK,
	BAR100_ERROR,
}BAR100_ReturnTypeDef_T;

typedef enum{
	Zero_At_Atm,
	Zero_At_1Bar,
	Zero_At_Vacuum,
}BAR100_Mode_T;

typedef struct{
	uint8_t I2C_No;
	uint8_t DevAdress;
}BAR100_DeviceParam_T;

typedef struct{
	BAR100_DeviceParam_T devParam;
	uint16_t PromData[10];
	uint32_t Temperature_RawData;
	uint32_t Pressure_RawData;
	uint16_t miliCelcius;
	uint16_t miliPressure;
	uint16_t producttype;
	uint32_t PMax;
	uint32_t PMin;
	BAR100_Mode_T mode;
}BAR100_Sensor_T;

extern BAR100_Sensor_T BAR100_1, BAR100_2, BAR100_3;

BAR100_ReturnTypeDef_T BAR100_Init(BAR100_Sensor_T *handle, uint8_t i2cNo, uint8_t devI2CAddress);
BAR100_ReturnTypeDef_T BAR100_Reset(BAR100_Sensor_T *handle);
BAR100_ReturnTypeDef_T BAR100_Write(BAR100_Sensor_T *handle, uint8_t command);
BAR100_ReturnTypeDef_T BAR100_Read(BAR100_Sensor_T *handle, uint8_t command, uint8_t *data);
BAR100_ReturnTypeDef_T BAR100_SendReceive(BAR100_Sensor_T *handle, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght);
BAR100_ReturnTypeDef_T BAR100_Get_AllPromData(BAR100_Sensor_T *handle);
BAR100_ReturnTypeDef_T BAR100_ReadPressure(BAR100_Sensor_T *handle);
BAR100_ReturnTypeDef_T BAR100_ReadTemperature(BAR100_Sensor_T *handle);
BAR100_ReturnTypeDef_T BAR100_StartPressureConversion(BAR100_Sensor_T *handle);
BAR100_ReturnTypeDef_T BAR100_StartTemperatureConversion(BAR100_Sensor_T *handle);
BAR100_ReturnTypeDef_T BAR100_ReadMeasData(BAR100_Sensor_T *handle, uint8_t command, uint8_t *buffer);
BAR100_ReturnTypeDef_T BAR100_Get_CustIds(BAR100_Sensor_T *handle);
BAR100_ReturnTypeDef_T BAR100_Get_ScalingValues(BAR100_Sensor_T *handle);
#endif /* SENSORS_BAR100_H_ */
