/**
 * \file I2C.c
 * \author Berkay Esenkaya (BAUROV Software Team)
 * \brief I2C communication peripheral porters
 * \date 24.02.2025
 *
 */

#include "I2C.h"
#include "main.h"
#include "stm32f7xx.h"

I2C_HandleTypeDef_T I2C_1, I2C_2, I2C_3;

/**********************************************************
 * GLOBAL FUNCTIONS
 *********************************************************/
/** Brief description which ends at this dot. Details follow
 *  here.
 */
void I2C_Init(void *handle, uint8_t I2CNo){
	I2C_HandleTypeDef_T *i2c;
	i2c = I2C_GetModule(I2CNo);
    i2c->handle = handle;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
I2C_ReturnTypeDef_T I2C_ReadWrite_Poll(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght){
	I2C_HandleTypeDef_T *i2c;
	HAL_StatusTypeDef res;
	uint8_t txAddr, rxAddr;
	txAddr = DevAddress<<1;
	rxAddr = ((DevAddress<<1) | 0x01);
	i2c = I2C_GetModule(I2CNo);
	if(rxLenght == 0){
		res = HAL_I2C_Master_Transmit(i2c->handle , txAddr, txBuff, txLenght,1000);
		return res == I2C_ERROR;
	}else{
		HAL_I2C_Master_Transmit(i2c->handle , txAddr, txBuff, txLenght,1000);
		HAL_Delay(1);
		res = HAL_I2C_Master_Receive(i2c->handle, rxAddr, rxBuff, rxLenght,1000);
		return res == I2C_ERROR;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
I2C_ReturnTypeDef_T I2C_ReadWrite_DMA(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght){
	I2C_HandleTypeDef_T *i2c;
	HAL_StatusTypeDef res;
	i2c = I2C_GetModule(I2CNo);
	if(rxLenght == 0){
		res = HAL_I2C_Master_Transmit_DMA(i2c->handle , DevAddress, txBuff, txLenght);
		I2C_WaitFlag(I2CNo);
		return res == I2C_ERROR;
	}else{
		HAL_I2C_Master_Transmit_DMA(i2c->handle , DevAddress, txBuff, txLenght);
		I2C_WaitFlag(I2CNo);
		res = HAL_I2C_Master_Receive_DMA(i2c->handle, DevAddress, rxBuff, rxLenght);
		I2C_WaitFlag(I2CNo);
		return res == I2C_ERROR;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
I2C_ReturnTypeDef_T I2C_ReadWrite_IT(uint8_t I2CNo, uint8_t DevAddress, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght){
	I2C_HandleTypeDef_T *i2c;
	HAL_StatusTypeDef res;
	i2c = I2C_GetModule(I2CNo);
	if(rxLenght == 0){
		res = HAL_I2C_Master_Transmit_IT(i2c->handle , DevAddress, txBuff, txLenght);
		I2C_WaitFlag(I2CNo);
		return res == I2C_ERROR;
	}else{
		HAL_I2C_Master_Transmit_IT(i2c->handle , DevAddress, txBuff, txLenght);
		I2C_WaitFlag(I2CNo);
		res = HAL_I2C_Master_Receive_IT(i2c->handle, DevAddress, rxBuff, rxLenght);
		I2C_WaitFlag(I2CNo);
		return res == I2C_ERROR;
	}
}

/**********************************************************
 * PRIVATE FUNCTIONS
 *********************************************************/
/** Brief description which ends at this dot. Details follow
 *  here.
 */
I2C_HandleTypeDef_T* I2C_GetModule(uint8_t I2CNo){
	switch(I2CNo){
		case I2CNO_1 : return &I2C_1; break;
		case I2CNO_2 : return &I2C_2; break;
		case I2CNO_3 : return &I2C_3; break;
		default : return NULL; break;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void I2C_WaitFlag(uint8_t I2CNo){
	I2C_HandleTypeDef_T *i2c;
	uint8_t i=0;
	i2c = I2C_GetModule(I2CNo);
	while((!i2c->flag) & (i<5)){
		HAL_Delay(1);
		i++;
	}
	i2c->flag = 0;
}

/**********************************************************
 * PRIVATE FUNCTIONS
 *********************************************************/
/** Brief description which ends at this dot. Details follow
 *  here.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  if(I2C_1.handle == hi2c)
	  I2C_1.flag = 1;
  else if(I2C_2.handle == hi2c)
  	  I2C_2.flag = 1;
  else if(I2C_3.handle == hi2c)
  	  I2C_3.flag = 1;
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterTxCpltCallback could be implemented in the user file
   */
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
	if(I2C_1.handle == hi2c)
		I2C_1.flag = 1;
	else if(I2C_2.handle == hi2c)
	  	I2C_2.flag = 1;
	else if(I2C_3.handle == hi2c)
	  	I2C_3.flag = 1;

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterTxCpltCallback could be implemented in the user file
   */
}



