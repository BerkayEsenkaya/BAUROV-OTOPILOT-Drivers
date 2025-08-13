/*
 * BAR100_CommPorter.c
 *
 *  Created on: Jul 21, 2025
 *      Author: Berkay Esenkaya
 */

#include <stdint.h>
#include "BAR100_CommPorter.h"
#include "I2C.h"

uint8_t BAR100_CommPorter_SendReceive(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght){
	return I2C_ReadWrite_Poll(I2CNo, DevAddress, txBuff, txLenght, rxBuff, rxLenght);
}
