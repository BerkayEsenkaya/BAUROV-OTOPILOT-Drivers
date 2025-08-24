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

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void IMU_Init(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo, uint8_t IMU_I2CNO, uint8_t IMU_I2CAdress, void* ResetGPIOPort, uint16_t ResetGPIOPin){
	BNO055_Sensor_T *imuSensor;
	imuSensor = IMU_GetSensorHandle(ImuSensorNo);

	BNO055_Init(imuSensor, IMU_I2CNO, IMU_I2CAdress, ResetGPIOPort, ResetGPIOPin);

	ImuHandle->CalcConst.ACC_1MpS2_LSB = imuSensor->CalcConst.ACC_1MpS2_LSB;
	ImuHandle->CalcConst.GYR_1RPS_LSB  = imuSensor->CalcConst.GYR_1RPS_LSB;
	ImuHandle->CalcConst.MAG_1uT_LSB   = imuSensor->CalcConst.MAG_1uT_LSB;

	HAL_Delay(25);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void IMU_Execute(IMU_TypeDef_T *ImuHandle, uint8_t ImuSensorNo){
	BNO055_Sensor_T *imuSensor;

	imuSensor = IMU_GetSensorHandle(ImuSensorNo);

	BNO055_Get_SysError(imuSensor);
	BNO055_Get_All_Data(imuSensor);

//	BNO055_Get_ACC_Data(imuSensor);
//	BNO055_Get_MAG_Data(imuSensor);
//	BNO055_Get_GYR_Data(imuSensor);

	IMU_PopulateData(ImuHandle, ImuSensorNo);

	MED_Filter_s16(&MED_IMU_ACC_X, ImuHandle->AllRawData.Accelerometer.X_Axis);
	AVG_Filter_s16(&AVG_IMU_ACC_X, MED_IMU_ACC_X.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_ACC_Y, ImuHandle->AllRawData.Accelerometer.Y_Axis);
	AVG_Filter_s16(&AVG_IMU_ACC_Y, MED_IMU_ACC_Y.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_ACC_Z, ImuHandle->AllRawData.Accelerometer.Z_Axis);
	AVG_Filter_s16(&AVG_IMU_ACC_Z, MED_IMU_ACC_Z.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_GYR_X, ImuHandle->AllRawData.Gyroscope.X_Axis);
	AVG_Filter_s16(&AVG_IMU_GYR_X, MED_IMU_GYR_X.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_GYR_Y, ImuHandle->AllRawData.Gyroscope.Y_Axis);
	AVG_Filter_s16(&AVG_IMU_GYR_Y, MED_IMU_GYR_Y.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_GYR_Z, ImuHandle->AllRawData.Gyroscope.Z_Axis);
	AVG_Filter_s16(&AVG_IMU_GYR_Z, MED_IMU_GYR_Z.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_MAG_X, ImuHandle->AllRawData.Magnetometer.X_Axis);
	AVG_Filter_s16(&AVG_IMU_MAG_X, MED_IMU_MAG_X.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_MAG_Y, ImuHandle->AllRawData.Magnetometer.Y_Axis);
	AVG_Filter_s16(&AVG_IMU_MAG_Y, MED_IMU_MAG_Y.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_MAG_Z, ImuHandle->AllRawData.Magnetometer.Z_Axis);
	AVG_Filter_s16(&AVG_IMU_MAG_Z, MED_IMU_MAG_Z.MED_FilteredData);


	MED_Filter_s16(&MED_IMU_HEADING, ImuHandle->AllRawData.Heading);
	AVG_Filter_s16(&AVG_IMU_HEADING, MED_IMU_HEADING.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_ROLL, ImuHandle->AllRawData.Roll);
	AVG_Filter_s16(&AVG_IMU_ROLL, MED_IMU_ROLL.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_PITCH, ImuHandle->AllRawData.Pitch);
	AVG_Filter_s16(&AVG_IMU_PITCH, MED_IMU_PITCH.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_QUA_X, ImuHandle->AllRawData.Qua.X_Axis);
	AVG_Filter_s16(&AVG_IMU_QUA_X, MED_IMU_QUA_X.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_QUA_Y, ImuHandle->AllRawData.Qua.Y_Axis);
	AVG_Filter_s16(&AVG_IMU_QUA_Y, MED_IMU_QUA_Y.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_QUA_Z, ImuHandle->AllRawData.Qua.Z_Axis);
	AVG_Filter_s16(&AVG_IMU_QUA_Z, MED_IMU_QUA_Z.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_LINACC_X, ImuHandle->AllRawData.LinAcc.X_Axis);
	AVG_Filter_s16(&AVG_IMU_LINACC_X, MED_IMU_LINACC_X.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_LINACC_Y, ImuHandle->AllRawData.LinAcc.Y_Axis);
	AVG_Filter_s16(&AVG_IMU_LINACC_Y, MED_IMU_LINACC_Y.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_LINACC_Z, ImuHandle->AllRawData.LinAcc.Z_Axis);
	AVG_Filter_s16(&AVG_IMU_LINACC_Z, MED_IMU_LINACC_Z.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_GRV_X, ImuHandle->AllRawData.Grv.X_Axis);
	AVG_Filter_s16(&AVG_IMU_GYR_X, MED_IMU_GRV_X.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_GRV_Y, ImuHandle->AllRawData.Grv.Y_Axis);
	AVG_Filter_s16(&AVG_IMU_GYR_Y, MED_IMU_GRV_Y.MED_FilteredData);
	MED_Filter_s16(&MED_IMU_GRV_Z, ImuHandle->AllRawData.Grv.Z_Axis);
	AVG_Filter_s16(&AVG_IMU_GYR_Z, MED_IMU_GRV_Z.MED_FilteredData);


	ImuHandle->FilteredData.Accelerometer.X_Axis = AVG_IMU_ACC_X.AVG_FilteredData;
	ImuHandle->FilteredData.Accelerometer.Y_Axis = AVG_IMU_ACC_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Accelerometer.Z_Axis = AVG_IMU_ACC_Z.AVG_FilteredData;
	ImuHandle->FilteredData.Gyroscope.X_Axis     = AVG_IMU_GYR_X.AVG_FilteredData;
	ImuHandle->FilteredData.Gyroscope.Y_Axis     = AVG_IMU_GYR_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Gyroscope.Z_Axis     = AVG_IMU_GYR_Z.AVG_FilteredData;
	ImuHandle->FilteredData.Magnetometer.X_Axis  = AVG_IMU_MAG_X.AVG_FilteredData;
	ImuHandle->FilteredData.Magnetometer.Y_Axis  = AVG_IMU_MAG_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Magnetometer.Z_Axis  = AVG_IMU_MAG_Z.AVG_FilteredData;

	ImuHandle->FilteredData.Heading = AVG_IMU_HEADING.AVG_FilteredData;
	ImuHandle->FilteredData.Roll = AVG_IMU_ROLL.AVG_FilteredData;
	ImuHandle->FilteredData.Pitch = AVG_IMU_PITCH.AVG_FilteredData;
	ImuHandle->FilteredData.Qua.W_Axis     = AVG_IMU_QUA_W.AVG_FilteredData;
	ImuHandle->FilteredData.Qua.X_Axis     = AVG_IMU_QUA_X.AVG_FilteredData;
	ImuHandle->FilteredData.Qua.Y_Axis     = AVG_IMU_QUA_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Qua.Z_Axis     = AVG_IMU_QUA_Z.AVG_FilteredData;
	ImuHandle->FilteredData.LinAcc.X_Axis  = AVG_IMU_LINACC_X.AVG_FilteredData;
	ImuHandle->FilteredData.LinAcc.Y_Axis  = AVG_IMU_LINACC_Y.AVG_FilteredData;
	ImuHandle->FilteredData.LinAcc.Z_Axis  = AVG_IMU_LINACC_Z.AVG_FilteredData;
	ImuHandle->FilteredData.Grv.X_Axis  = AVG_IMU_GRV_X.AVG_FilteredData;
	ImuHandle->FilteredData.Grv.Y_Axis  = AVG_IMU_GRV_Y.AVG_FilteredData;
	ImuHandle->FilteredData.Grv.Z_Axis  = AVG_IMU_GRV_Z.AVG_FilteredData;

	ImuHandle->CalculatedData.Accelerometer.X_Axis = (float)(ImuHandle->FilteredData.Accelerometer.X_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB);
	ImuHandle->CalculatedData.Accelerometer.Y_Axis = (float)(ImuHandle->FilteredData.Accelerometer.Y_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB);
	ImuHandle->CalculatedData.Accelerometer.Z_Axis = (float)(ImuHandle->FilteredData.Accelerometer.Z_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB);
	ImuHandle->CalculatedData.Gyroscope.X_Axis     = (float)(ImuHandle->FilteredData.Gyroscope.X_Axis / ImuHandle->CalcConst.GYR_1RPS_LSB / 16 * 3.1415);
	ImuHandle->CalculatedData.Gyroscope.Y_Axis     = (float)(ImuHandle->FilteredData.Gyroscope.Y_Axis / ImuHandle->CalcConst.GYR_1RPS_LSB / 16 * 3.1415);
	ImuHandle->CalculatedData.Gyroscope.Z_Axis     = (float)(ImuHandle->FilteredData.Gyroscope.Z_Axis / ImuHandle->CalcConst.GYR_1RPS_LSB / 16 * 3.1415);
	ImuHandle->CalculatedData.Magnetometer.X_Axis  = (float)((ImuHandle->FilteredData.Magnetometer.X_Axis / ImuHandle->CalcConst.MAG_1uT_LSB * 0.000001));
	ImuHandle->CalculatedData.Magnetometer.Y_Axis  = (float)((ImuHandle->FilteredData.Magnetometer.Y_Axis / ImuHandle->CalcConst.MAG_1uT_LSB * 0.000001));
	ImuHandle->CalculatedData.Magnetometer.Z_Axis  = (float)((ImuHandle->FilteredData.Magnetometer.Z_Axis / ImuHandle->CalcConst.MAG_1uT_LSB * 0.000001));
	ImuHandle->CalculatedData.Heading = (float)((ImuHandle->FilteredData.Heading / 16.0));
	ImuHandle->CalculatedData.Roll = (float)((ImuHandle->FilteredData.Roll));
	ImuHandle->CalculatedData.Pitch = (float)((ImuHandle->FilteredData.Pitch));
	ImuHandle->CalculatedData.Qua.W_Axis = (float)((ImuHandle->FilteredData.Qua.W_Axis / 16384.0 ));
	ImuHandle->CalculatedData.Qua.X_Axis = (float)((ImuHandle->FilteredData.Qua.X_Axis / 16384.0 ));
	ImuHandle->CalculatedData.Qua.Y_Axis = (float)((ImuHandle->FilteredData.Qua.Y_Axis / 16384.0 ));
	ImuHandle->CalculatedData.Qua.Z_Axis = (float)((ImuHandle->FilteredData.Qua.Z_Axis / 16384.0 ));
	ImuHandle->CalculatedData.LinAcc.X_Axis = (float)((ImuHandle->FilteredData.LinAcc.X_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB));
	ImuHandle->CalculatedData.LinAcc.Y_Axis = (float)((ImuHandle->FilteredData.LinAcc.Y_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB));
	ImuHandle->CalculatedData.LinAcc.Z_Axis = (float)((ImuHandle->FilteredData.LinAcc.Z_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB));
	ImuHandle->CalculatedData.Grv.X_Axis = (float)((ImuHandle->FilteredData.Grv.X_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB));
	ImuHandle->CalculatedData.Grv.Y_Axis = (float)((ImuHandle->FilteredData.Grv.Y_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB));
	ImuHandle->CalculatedData.Grv.Z_Axis = (float)((ImuHandle->FilteredData.Grv.Z_Axis / ImuHandle->CalcConst.ACC_1MpS2_LSB));
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
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

	ImuHandle->AllRawData.Heading  = imuSensor->ImuData.Heading;
	ImuHandle->AllRawData.Roll  =    imuSensor->ImuData.Roll;
	ImuHandle->AllRawData.Pitch  =   imuSensor->ImuData.Pitch;
	ImuHandle->AllRawData.Qua.X_Axis  = imuSensor->ImuData.QUA_X;
	ImuHandle->AllRawData.Qua.Y_Axis  = imuSensor->ImuData.QUA_Y;
	ImuHandle->AllRawData.Qua.Z_Axis  = imuSensor->ImuData.QUA_Z;
	ImuHandle->AllRawData.LinAcc.X_Axis  = imuSensor->ImuData.LINACC_X;
	ImuHandle->AllRawData.LinAcc.Y_Axis  = imuSensor->ImuData.LINACC_Y;
	ImuHandle->AllRawData.LinAcc.Z_Axis  = imuSensor->ImuData.LINACC_Z;

	ImuHandle->AllRawData.Grv.X_Axis  = imuSensor->ImuData.GRV_X;
	ImuHandle->AllRawData.Grv.Y_Axis  = imuSensor->ImuData.GRV_Y;
	ImuHandle->AllRawData.Grv.Z_Axis  = imuSensor->ImuData.GRV_Z;

}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_Sensor_T* IMU_GetSensorHandle(uint8_t sensorNo){
	switch(sensorNo){
		case 1 : return &BNO055_Sensor_1;break;
		case 2 : break;
		case 3 : break;
		default : return &BNO055_Sensor_1;
	}
}
