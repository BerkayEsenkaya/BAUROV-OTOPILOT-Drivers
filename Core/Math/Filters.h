/*
 * Filters.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Berkay Esenkaya
 */

#ifndef MATH_FILTERS_H_
#define MATH_FILTERS_H_

#define AVG_FilterBufferSize 4
#define MED_FilterBufferSize 5 //tek sayı olmalı

typedef struct{
	int16_t AVG_FilterBuffer[AVG_FilterBufferSize];
	int16_t AVG_FilteredData;
	int16_t AVG_FilteredPreData;
	uint16_t AVG_FilterCounter;
}AVG_Filter_s16_TypeDef_T;

typedef struct{
	int16_t MED_FilterBuffer[MED_FilterBufferSize];
	int16_t MED_FilteredData;
	int16_t MED_FilteredPreData;
	uint16_t MED_FilterCounter;
}MED_Filter_s16_TypeDef_T;

typedef struct{
	uint32_t AVG_FilterBuffer[AVG_FilterBufferSize];
	uint32_t AVG_FilteredData;
	uint32_t AVG_FilteredPreData;
	uint16_t AVG_FilterCounter;
}AVG_Filter_u32_TypeDef_T;

typedef struct{
	uint32_t MED_FilterBuffer[MED_FilterBufferSize];
	uint32_t MED_FilteredData;
	uint32_t MED_FilteredPreData;
	uint16_t MED_FilterCounter;
}MED_Filter_u32_TypeDef_T;

extern AVG_Filter_s16_TypeDef_T AVG_IMU_ACC_X, AVG_IMU_ACC_Y, AVG_IMU_ACC_Z, AVG_IMU_GYR_X, AVG_IMU_GYR_Y, AVG_IMU_GYR_Z, AVG_IMU_MAG_X, AVG_IMU_MAG_Y, AVG_IMU_MAG_Z;
extern MED_Filter_s16_TypeDef_T MED_IMU_ACC_X, MED_IMU_ACC_Y, MED_IMU_ACC_Z, MED_IMU_GYR_X, MED_IMU_GYR_Y, MED_IMU_GYR_Z, MED_IMU_MAG_X, MED_IMU_MAG_Y, MED_IMU_MAG_Z;
extern AVG_Filter_u32_TypeDef_T AVG_PressureSensor_1;
extern MED_Filter_u32_TypeDef_T MED_PressureSensor_1;

extern AVG_Filter_s16_TypeDef_T AVG_IMU_HEADING, AVG_IMU_PITCH, AVG_IMU_ROLL, AVG_IMU_QUA_W, AVG_IMU_QUA_X, AVG_IMU_QUA_Y, AVG_IMU_QUA_Z, AVG_IMU_LINACC_X, AVG_IMU_LINACC_Y, AVG_IMU_LINACC_Z, AVG_IMU_GRV_X, AVG_IMU_GRV_Y, AVG_IMU_GRV_Z;
extern MED_Filter_s16_TypeDef_T MED_IMU_HEADING, MED_IMU_PITCH, MED_IMU_ROLL, MED_IMU_QUA_W, MED_IMU_QUA_X, MED_IMU_QUA_Y, MED_IMU_QUA_Z, MED_IMU_LINACC_X, MED_IMU_LINACC_Y, MED_IMU_LINACC_Z, MED_IMU_GRV_X, MED_IMU_GRV_Y, MED_IMU_GRV_Z;

void AVG_Filter_s16(AVG_Filter_s16_TypeDef_T *handle, int16_t data);
void MED_Filter_s16(MED_Filter_s16_TypeDef_T *handle, int16_t data);
void AVG_Filter_u32(AVG_Filter_u32_TypeDef_T *handle, uint32_t data);
void MED_Filter_u32(MED_Filter_u32_TypeDef_T *handle, uint32_t data);
#endif /* MATH_FILTERS_H_ */
