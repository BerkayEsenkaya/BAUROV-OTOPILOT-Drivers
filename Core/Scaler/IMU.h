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
	int16_t X_Axis;
	int16_t Y_Axis;
	int16_t Z_Axis;
}IMU_AxisSigned_T;

typedef struct{
	int16_t W_Axis;
	int16_t X_Axis;
	int16_t Y_Axis;
	int16_t Z_Axis;
}IMU_QuaAxisSigned_T;

typedef struct{
	IMU_AxisSigned_T Accelerometer;
	IMU_AxisSigned_T Gyroscope;
	IMU_AxisSigned_T Magnetometer;
	float Heading;
	float Roll;
	float Pitch;
	IMU_QuaAxisSigned_T Qua;
	IMU_AxisSigned_T LinAcc;
	IMU_AxisSigned_T Grv;
}IMU_SignedData_T;

typedef struct{
	float X_Axis;
	float Y_Axis;
	float Z_Axis;
}IMU_AxisFloat_T;

typedef struct{
	float W_Axis;
	float X_Axis;
	float Y_Axis;
	float Z_Axis;
}IMU_QUAAxisFloat_T;

typedef struct{
	IMU_AxisFloat_T Accelerometer;
	IMU_AxisFloat_T Gyroscope;
	IMU_AxisFloat_T Magnetometer;
	float Heading;
	float Roll;
	float Pitch;
	IMU_QUAAxisFloat_T Qua;
	IMU_AxisFloat_T LinAcc;
	IMU_AxisFloat_T Grv;
}IMU_FloatData_T;

typedef struct{
	float ACC_1MpS2_LSB;
	float MAG_1uT_LSB;
	float GYR_1RPS_LSB;
}IMU_CalculateConstants_T;

typedef struct{
	IMU_SignedData_T AllRawData;
	IMU_SignedData_T FilteredData;
	IMU_FloatData_T CalculatedData;
	IMU_CalculateConstants_T CalcConst;
}IMU_TypeDef_T;

extern IMU_TypeDef_T IMU_1;

void IMU_PopulateData(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo);
void IMU_Execute(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo);
void IMU_Init(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo, uint8_t IMU_I2CNO, uint8_t IMU_I2CAdress, void* ResetGPIOPort, uint16_t ResetGPIOPin);
BNO055_Sensor_T* IMU_GetSensorHandle(uint8_t sensorNo);

#endif /* SCALER_IMU_H_ */
