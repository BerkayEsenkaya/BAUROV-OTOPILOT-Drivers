/*
 * IMU.c
 *
 *  Created on: Jul 24, 2025
 *      Author: Berkay Esenkaya
 */

#include <stdint.h>
#include "main.h"
#include "BNO055.h"
#include "IMU.h"
#include "Filters.h"

IMU_TypeDef_T IMU_1;

void IMU_Init(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo, uint8_t IMU_I2CNO, uint8_t IMU_I2CAdress, void* ResetGPIOPort, uint16_t ResetGPIOPin){
	BNO055_Sensor_T *imuSensor;
	imuSensor = IMU_GetSensorHandle(ImuSensorNo);

	BNO055_Init(imuSensor, IMU_I2CNO, IMU_I2CAdress, ResetGPIOPort, ResetGPIOPin);

	ImuHandle->CalcConst.ACC_1MpS2_LSB = imuSensor->CalcConst.ACC_1MpS2_LSB;
	ImuHandle->CalcConst.GYR_1RPS_LSB  = imuSensor->CalcConst.GYR_1RPS_LSB;
	ImuHandle->CalcConst.MAG_1uT_LSB   = imuSensor->CalcConst.MAG_1uT_LSB;

	HAL_Delay(250);
}

void IMU_Execute(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo){
	BNO055_Sensor_T *imuSensor;

	imuSensor = IMU_GetSensorHandle(ImuSensorNo);

	BNO055_Get_SysError(imuSensor);
	BNO055_Get_ACC_Data(imuSensor);
	BNO055_Get_MAG_Data(imuSensor);
	BNO055_Get_GYR_Data(imuSensor);

	IMU_PopulateData(ImuHandle, ImuSensorNo);

	MED_Filter(&MED_IMU_ACC_X, ImuHandle->AllRawData.Accelerometer.X_Axis);
	AVG_Filter(&AVG_IMU_ACC_X, MED_IMU_ACC_X.MED_FilteredData);
	MED_Filter(&MED_IMU_ACC_Y, ImuHandle->AllRawData.Accelerometer.Y_Axis);
	AVG_Filter(&AVG_IMU_ACC_Y, MED_IMU_ACC_Y.MED_FilteredData);
	MED_Filter(&MED_IMU_ACC_Z, ImuHandle->AllRawData.Accelerometer.Z_Axis);
	AVG_Filter(&AVG_IMU_ACC_Z, MED_IMU_ACC_Z.MED_FilteredData);
	MED_Filter(&MED_IMU_GYR_X, ImuHandle->AllRawData.Gyroscope.X_Axis);
	AVG_Filter(&AVG_IMU_GYR_X, MED_IMU_GYR_X.MED_FilteredData);
	MED_Filter(&MED_IMU_GYR_Y, ImuHandle->AllRawData.Gyroscope.Y_Axis);
	AVG_Filter(&AVG_IMU_GYR_Y, MED_IMU_GYR_Y.MED_FilteredData);
	MED_Filter(&MED_IMU_GYR_Z, ImuHandle->AllRawData.Gyroscope.Z_Axis);
	AVG_Filter(&AVG_IMU_GYR_Z, MED_IMU_GYR_Z.MED_FilteredData);
	MED_Filter(&MED_IMU_MAG_X, ImuHandle->AllRawData.Magnetometer.X_Axis);
	AVG_Filter(&AVG_IMU_MAG_X, MED_IMU_MAG_X.MED_FilteredData);
	MED_Filter(&MED_IMU_MAG_Y, ImuHandle->AllRawData.Magnetometer.Y_Axis);
	AVG_Filter(&AVG_IMU_MAG_Y, MED_IMU_MAG_Y.MED_FilteredData);
	MED_Filter(&MED_IMU_MAG_Z, ImuHandle->AllRawData.Magnetometer.Z_Axis);
	AVG_Filter(&AVG_IMU_MAG_Z, MED_IMU_MAG_Z.MED_FilteredData);

	ImuHandle->FilteredData.Accelerometer.X_Axis = AVG_IMU_ACC_X.AVG_FilteredData;
	ImuHandle->FilteredData.Accelerometer.Y_Axis = AVG_IMU_ACC_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Accelerometer.Z_Axis = AVG_IMU_ACC_Z.AVG_FilteredData;
	ImuHandle->FilteredData.Gyroscope.X_Axis     = AVG_IMU_GYR_X.AVG_FilteredData;
	ImuHandle->FilteredData.Gyroscope.Y_Axis     = AVG_IMU_GYR_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Gyroscope.Z_Axis     = AVG_IMU_GYR_Z.AVG_FilteredData;
	ImuHandle->FilteredData.Magnetometer.X_Axis  = AVG_IMU_MAG_X.AVG_FilteredData;
	ImuHandle->FilteredData.Magnetometer.Y_Axis  = AVG_IMU_MAG_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Magnetometer.Z_Axis  = AVG_IMU_MAG_Z.AVG_FilteredData;


	ImuHandle->CalculatedData.Accelerometer.X_Axis = (float)(ImuHandle->FilteredData.Accelerometer.X_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB);
	ImuHandle->CalculatedData.Accelerometer.Y_Axis = (float)(ImuHandle->FilteredData.Accelerometer.Y_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB);
	ImuHandle->CalculatedData.Accelerometer.Z_Axis = (float)(ImuHandle->FilteredData.Accelerometer.Z_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB);
	ImuHandle->CalculatedData.Gyroscope.X_Axis     = (float)(ImuHandle->FilteredData.Gyroscope.X_Axis / ImuHandle->CalcConst.GYR_1RPS_LSB);
	ImuHandle->CalculatedData.Gyroscope.Y_Axis     = (float)(ImuHandle->FilteredData.Gyroscope.Y_Axis / ImuHandle->CalcConst.GYR_1RPS_LSB);
	ImuHandle->CalculatedData.Gyroscope.Z_Axis     = (float)(ImuHandle->FilteredData.Gyroscope.Z_Axis / ImuHandle->CalcConst.GYR_1RPS_LSB);
	ImuHandle->CalculatedData.Magnetometer.X_Axis  = (float)((ImuHandle->FilteredData.Magnetometer.X_Axis / ImuHandle->CalcConst.MAG_1uT_LSB));
	ImuHandle->CalculatedData.Magnetometer.Y_Axis  = (float)((ImuHandle->FilteredData.Magnetometer.Y_Axis / ImuHandle->CalcConst.MAG_1uT_LSB));
	ImuHandle->CalculatedData.Magnetometer.Z_Axis  = (float)((ImuHandle->FilteredData.Magnetometer.Z_Axis / ImuHandle->CalcConst.MAG_1uT_LSB));
//
//	ImuHandle->CalculatedData.Accelerometer.X_Axis = (int16_t)IMU_1.AllRawData.Accelerometer.X_Axis;
//	ImuHandle->CalculatedData.Accelerometer.Y_Axis = (int16_t)IMU_1.AllRawData.Accelerometer.Y_Axis;
//	ImuHandle->CalculatedData.Accelerometer.Z_Axis = (int16_t)IMU_1.AllRawData.Accelerometer.Z_Axis;
//	ImuHandle->CalculatedData.Gyroscope.X_Axis     = (int16_t)IMU_1.AllRawData.Gyroscope.X_Axis;
//	ImuHandle->CalculatedData.Gyroscope.Y_Axis     = (int16_t)IMU_1.AllRawData.Gyroscope.Y_Axis;
//	ImuHandle->CalculatedData.Gyroscope.Z_Axis     = (int16_t)IMU_1.AllRawData.Gyroscope.Z_Axis;
//	ImuHandle->CalculatedData.Magnetometer.X_Axis  = (int16_t)IMU_1.AllRawData.Magnetometer.X_Axis;
//	ImuHandle->CalculatedData.Magnetometer.Y_Axis  = (int16_t)IMU_1.AllRawData.Magnetometer.Y_Axis;
//	ImuHandle->CalculatedData.Magnetometer.Z_Axis  = (int16_t)IMU_1.AllRawData.Magnetometer.Z_Axis;

}

void IMU_PopulateData(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo){
	BNO055_Sensor_T *imuSensor;
	imuSensor = IMU_GetSensorHandle(ImuSensorNo);

	ImuHandle->AllRawData.Accelerometer.X_Axis = (int16_t)imuSensor->ImuData.ACC_X;
	ImuHandle->AllRawData.Accelerometer.Y_Axis = (int16_t)imuSensor->ImuData.ACC_Y;
	ImuHandle->AllRawData.Accelerometer.Z_Axis = (int16_t)imuSensor->ImuData.ACC_Z;
	ImuHandle->AllRawData.Gyroscope.X_Axis     = imuSensor->ImuData.GYR_X;
	ImuHandle->AllRawData.Gyroscope.Y_Axis     = imuSensor->ImuData.GYR_Y;
	ImuHandle->AllRawData.Gyroscope.Z_Axis     = imuSensor->ImuData.GYR_Z;
	ImuHandle->AllRawData.Magnetometer.X_Axis  = imuSensor->ImuData.MAG_X;
	ImuHandle->AllRawData.Magnetometer.Y_Axis  = imuSensor->ImuData.MAG_Y;
	ImuHandle->AllRawData.Magnetometer.Z_Axis  = imuSensor->ImuData.MAG_Z;
}

BNO055_Sensor_T* IMU_GetSensorHandle(uint8_t sensorNo){
	switch(sensorNo){
		case 1 : return &BNO055_Sensor_1;break;
		case 2 : break;
		case 3 : break;
		default : return &BNO055_Sensor_1;
	}
}
