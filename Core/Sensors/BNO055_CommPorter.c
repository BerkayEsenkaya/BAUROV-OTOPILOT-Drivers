/*
 * BNO055_CommPorter.c
 *
 *  Created on: Jul 21, 2025
 *      Author: ggize
 */

#include <stdint.h>
#include "I2C.h"

uint8_t BNO055_CommPorter_SendReceive(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght){
	return I2C_ReadWrite_Poll(I2CNo, DevAddress, txBuff, txLenght, rxBuff, rxLenght);
//	return I2C_ReadWrite_IT(I2CNo, DevAddress, txBuff, txLenght, rxBuff, rxLenght);
//	return I2C_ReadWrite_DMA(I2CNo, DevAddress, txBuff, txLenght, rxBuff, rxLenght);
}
