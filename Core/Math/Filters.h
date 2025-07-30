/*
 * Filters.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Berkay Esenkaya
 */

#ifndef MATH_FILTERS_H_
#define MATH_FILTERS_H_

#define AVG_FilterBufferSize 10
#define MED_FilterBufferSize 7 //tek sayı olmalı

typedef struct{
	int16_t AVG_FilterBuffer[AVG_FilterBufferSize];
	int16_t AVG_FilteredData;
	int16_t AVG_FilteredPreData;
	uint16_t AVG_FilterCounter;
}AVG_FilterTypeDef_T;

typedef struct{
	int16_t MED_FilterBuffer[MED_FilterBufferSize];
	int16_t MED_FilteredData;
	int16_t MED_FilteredPreData;
	uint16_t MED_FilterCounter;
}MED_FilterTypeDef_T;

extern AVG_FilterTypeDef_T AVG_IMU_ACC_X, AVG_IMU_ACC_Y, AVG_IMU_ACC_Z, AVG_IMU_GYR_X, AVG_IMU_GYR_Y, AVG_IMU_GYR_Z, AVG_IMU_MAG_X, AVG_IMU_MAG_Y, AVG_IMU_MAG_Z;
extern MED_FilterTypeDef_T MED_IMU_ACC_X, MED_IMU_ACC_Y, MED_IMU_ACC_Z, MED_IMU_GYR_X, MED_IMU_GYR_Y, MED_IMU_GYR_Z, MED_IMU_MAG_X, MED_IMU_MAG_Y, MED_IMU_MAG_Z;

void AVG_Filter(AVG_FilterTypeDef_T *handle, uint32_t data);
void MED_Filter(MED_FilterTypeDef_T *handle, uint32_t data);
#endif /* MATH_FILTERS_H_ */
