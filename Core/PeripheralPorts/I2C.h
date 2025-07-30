/*
 * I2C.h
 *
 *  Created on: Jul 21, 2025
 *      Author: Berkay Esenkaya
 */

#ifndef I2C_H
#define I2C_H

#include "stdint.h"

#define I2CNO_1 (1)
#define I2CNO_2 (2)
#define I2CNO_3 (3)

typedef struct{
	void *handle;
	volatile uint8_t flag;
}I2C_HandleTypeDef_T;

typedef enum{
	I2C_OK,
	I2C_ERROR,
}I2C_ReturnTypeDef_T;

void I2C_Init(void *handle, uint8_t I2CNo);
I2C_ReturnTypeDef_T I2C_ReadWrite_Poll(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght);
I2C_ReturnTypeDef_T I2C_ReadWrite_IT(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght);
I2C_ReturnTypeDef_T I2C_ReadWrite_DMA(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght);
I2C_HandleTypeDef_T* I2C_GetModule(uint8_t I2CNo);
void I2C_WaitFlag(uint8_t I2CNo);


#endif /* PERIPHERALPORTS_I2C_H_ */
