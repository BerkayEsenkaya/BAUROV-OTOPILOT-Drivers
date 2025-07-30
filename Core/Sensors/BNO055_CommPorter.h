/*
 * BNO055_CommPorter.h
 *
 *  Created on: Jul 21, 2025
 *      Author: ggize
 */

#ifndef SENSORS_BNO055_COMMPORTER_H_
#define SENSORS_BNO055_COMMPORTER_H_

uint8_t BNO055_CommPorter_SendReceive(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght);

#endif /* SENSORS_BNO055_COMMPORTER_H_ */
