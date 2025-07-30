/*
 * IMU.h
 *
 *  Created on: Jul 24, 2025
 *      Author: Berkay Esenkaya
 */

#ifndef SCALER_IMU_H_
#define SCALER_IMU_H_

#define IMU_BNO055_1_I2C_Address 0x28

typedef struct{
	uint16_t X_Axis;
	uint16_t Y_Axis;
	uint16_t Z_Axis;
}IMU_AxisUnsigned_T;

typedef struct{
	IMU_AxisUnsigned_T Accelerometer;
	IMU_AxisUnsigned_T Gyroscope;
	IMU_AxisUnsigned_T Magnetometer;
}IMU_UnsignedData_T;

typedef struct{
	int16_t X_Axis;
	int16_t Y_Axis;
	int16_t Z_Axis;
}IMU_AxisSigned_T;

typedef struct{
	IMU_AxisSigned_T Accelerometer;
	IMU_AxisSigned_T Gyroscope;
	IMU_AxisSigned_T Magnetometer;
}IMU_SignedData_T;

typedef struct{
	uint16_t ACC_1MpS2_LSB;
	uint16_t MAG_1uT_LSB;
	uint16_t GYR_1RPS_LSB;
}IMU_CalculateConstants_T;

typedef struct{
	IMU_UnsignedData_T AllRawData;
	IMU_UnsignedData_T FilteredData;
	IMU_SignedData_T CalculatedData;
	IMU_CalculateConstants_T CalcConst;
}IMU_TypeDef_T;

extern IMU_TypeDef_T IMU_1;

void IMU_PopulateData(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo);
void IMU_Execute(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo);
void IMU_Init(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo, uint8_t IMU_I2CNO, uint8_t IMU_I2CAdress, void* ResetGPIOPort, uint16_t ResetGPIOPin);
BNO055_Sensor_T* IMU_GetSensorHandle(uint8_t sensorNo);

#endif /* SCALER_IMU_H_ */
