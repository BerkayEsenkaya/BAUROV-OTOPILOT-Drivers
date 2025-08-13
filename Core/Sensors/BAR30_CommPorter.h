/*
 * BAR30_CommPorter.h
 *
 *  Created on: Aug 13, 2025
 *      Author: Berkay Esenkaya
 */

#ifndef SENSORS_BAR30_COMMPORTER_H_
#define SENSORS_BAR30_COMMPORTER_H_

uint8_t BAR30_CommPorter_SendReceive(uint8_t I2CNo, uint8_t DevAdress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght);

#endif /* SENSORS_BAR30_COMMPORTER_H_ */
