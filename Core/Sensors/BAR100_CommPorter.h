/*
 * BAR100_CommPorter.h
 *
 *  Created on: Jul 21, 2025
 *      Author: ggize
 */

#ifndef SENSORS_BAR100_COMMPORTER_H_
#define SENSORS_BAR100_COMMPORTER_H_

uint8_t BAR30_CommPorter_SendReceive(uint8_t I2CNo, uint8_t DevAdress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght);

#endif /* SENSORS_BAR100_COMMPORTER_H_ */
