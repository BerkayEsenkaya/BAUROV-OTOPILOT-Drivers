/*
 * Filters.c
 *
 *  Created on: Jul 25, 2025
 *      Author: Berkay Esenkaya
 */
/**********************************************************
 * INCLUDES
 *********************************************************/
#include <stdint.h>
#include <string.h>
#include "Filters.h"

/**********************************************************
 * GLOBAL VARIABLES
 *********************************************************/
AVG_Filter_s16_TypeDef_T AVG_IMU_ACC_X, AVG_IMU_ACC_Y, AVG_IMU_ACC_Z, AVG_IMU_GYR_X, AVG_IMU_GYR_Y, AVG_IMU_GYR_Z, AVG_IMU_MAG_X, AVG_IMU_MAG_Y, AVG_IMU_MAG_Z;
MED_Filter_s16_TypeDef_T MED_IMU_ACC_X, MED_IMU_ACC_Y, MED_IMU_ACC_Z, MED_IMU_GYR_X, MED_IMU_GYR_Y, MED_IMU_GYR_Z, MED_IMU_MAG_X, MED_IMU_MAG_Y, MED_IMU_MAG_Z;

AVG_Filter_s16_TypeDef_T AVG_IMU_HEADING, AVG_IMU_PITCH, AVG_IMU_ROLL, AVG_IMU_QUA_W, AVG_IMU_QUA_X, AVG_IMU_QUA_Y, AVG_IMU_QUA_Z, AVG_IMU_LINACC_X, AVG_IMU_LINACC_Y, AVG_IMU_LINACC_Z, AVG_IMU_GRV_X, AVG_IMU_GRV_Y, AVG_IMU_GRV_Z;
MED_Filter_s16_TypeDef_T MED_IMU_HEADING, MED_IMU_PITCH, MED_IMU_ROLL, MED_IMU_QUA_W, MED_IMU_QUA_X, MED_IMU_QUA_Y, MED_IMU_QUA_Z, MED_IMU_LINACC_X, MED_IMU_LINACC_Y, MED_IMU_LINACC_Z, MED_IMU_GRV_X, MED_IMU_GRV_Y, MED_IMU_GRV_Z;

AVG_Filter_u32_TypeDef_T AVG_PressureSensor_1;
MED_Filter_u32_TypeDef_T MED_PressureSensor_1;

/**********************************************************
 * GLOBAL FUNCTIONS
 *********************************************************/
/** Brief description which ends at this dot. Details follow
 *  here.
 */
void AVG_Filter_s16(AVG_Filter_s16_TypeDef_T *handle, int16_t data){
	int64_t sum = 0;
	handle->AVG_FilterBuffer[handle->AVG_FilterCounter++] = data;
	if(handle->AVG_FilterCounter == AVG_FilterBufferSize){
		for(uint8_t i=0; i<AVG_FilterBufferSize; i++)
			sum += handle->AVG_FilterBuffer[i];

		handle->AVG_FilteredData = (int16_t)(sum/AVG_FilterBufferSize);
		handle->AVG_FilteredPreData = handle->AVG_FilteredData;
		handle->AVG_FilterCounter = 0;
		memset(handle->AVG_FilterBuffer, 0, sizeof(handle->AVG_FilterBuffer));
	}else{
		handle->AVG_FilteredData=handle->AVG_FilteredPreData;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
static void MED_Filter_SortArray_s16(int16_t* arr, uint8_t size) {
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = i + 1; j < size; j++) {
            if (arr[j] < arr[i]) {
                int16_t tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void MED_Filter_s16(MED_Filter_s16_TypeDef_T *handle, int16_t data){
	handle->MED_FilterBuffer[handle->MED_FilterCounter++] = data;
	if(handle->MED_FilterCounter == MED_FilterBufferSize){
		MED_Filter_SortArray_s16(handle->MED_FilterBuffer, MED_FilterBufferSize);
		handle->MED_FilteredData = handle->MED_FilterBuffer[(MED_FilterBufferSize-1)/2];
		handle->MED_FilteredPreData = handle->MED_FilteredData;
		handle->MED_FilterCounter = 0;
		memset(handle->MED_FilterBuffer, 0, sizeof(handle->MED_FilterBuffer));
	}else{
		handle->MED_FilteredData=handle->MED_FilteredPreData;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void AVG_Filter_u32(AVG_Filter_u32_TypeDef_T *handle, uint32_t data){
	int64_t sum = 0;
	handle->AVG_FilterBuffer[handle->AVG_FilterCounter++] = data;
	if(handle->AVG_FilterCounter == AVG_FilterBufferSize){
		for(uint8_t i=0; i<AVG_FilterBufferSize; i++)
			sum += handle->AVG_FilterBuffer[i];

		handle->AVG_FilteredData = (uint32_t)(sum/AVG_FilterBufferSize);
		handle->AVG_FilteredPreData = handle->AVG_FilteredData;
		handle->AVG_FilterCounter = 0;
		memset(handle->AVG_FilterBuffer, 0, sizeof(handle->AVG_FilterBuffer));
	}else{
		handle->AVG_FilteredData=handle->AVG_FilteredPreData;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
static void MED_Filter_SortArray_u32(uint32_t* arr, uint8_t size) {
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = i + 1; j < size; j++) {
            if (arr[j] < arr[i]) {
                int16_t tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void MED_Filter_u32(MED_Filter_u32_TypeDef_T *handle, uint32_t data){
	handle->MED_FilterBuffer[handle->MED_FilterCounter++] = data;
	if(handle->MED_FilterCounter == MED_FilterBufferSize){
		MED_Filter_SortArray_u32(handle->MED_FilterBuffer, MED_FilterBufferSize);
		handle->MED_FilteredData = handle->MED_FilterBuffer[(MED_FilterBufferSize-1)/2];
		handle->MED_FilteredPreData = handle->MED_FilteredData;
		handle->MED_FilterCounter = 0;
		memset(handle->MED_FilterBuffer, 0, sizeof(handle->MED_FilterBuffer));
	}else{
		handle->MED_FilteredData=handle->MED_FilteredPreData;
	}
}
