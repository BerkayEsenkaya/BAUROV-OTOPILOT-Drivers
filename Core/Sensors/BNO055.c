/*
 * BNO055.c
 *
 *  Created on: Jul 21, 2025
 *      Author: Berkay Esenkaya
 */
#include <stdint.h>
#include "main.h"
#include "BNO055.h"
#include "BNO055_CommPorter.h"

BNO055_Sensor_T BNO055_Sensor_1;

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void BNO055_Init(BNO055_Sensor_T *handle, uint8_t I2C_No, uint8_t I2C_Adress, void* ResetGPIOPort, uint16_t ResetGPIOPin){
	handle->I2C_No = I2C_No;
	handle->Chip_I2C_Address = I2C_Adress;
	handle->ResetPort = ResetGPIOPort;
	handle->ResetPin = ResetGPIOPin;
	handle->CalcConst.ACC_1MpS2_LSB = 100;
	handle->CalcConst.GYR_1RPS_LSB = 900;
	handle->CalcConst.MAG_1uT_LSB = 16;
	handle->CalcConst.ACCRange_G = 4;
	handle->CalcConst.GYRRange_DPS = 500;

	BNO055_Reset(handle);

	BNO055_Set_OperationMode(handle, BNO055_DATA_OPR_MODE_NDOF);
	HAL_Delay(25);

//    while(1){
//    	BNO055_GetCalibrationState(handle);
//    	if((handle->CalibState == 0xFF)){
//    		HAL_Delay(5);
//    		BNO055_GetCalibrationData(handle);
//    		break;
//    	}
//    }

    BNO055_Set_OperationMode(handle, BNO055_DATA_OPR_MODE_OPR);
    HAL_Delay(25);

	BNO055_Set_PowerMode(handle, BNO055_DATA_PWR_MODE_NORMAL);
//	BNO055_GetCalibrationData(handle);
	BNO055_Set_DataUnit(handle, BNO055_DATA_UNIT_ACC_mG | BNO055_DATA_UNIT_ORI_AND);

	BNO055_Get_DataUnit(handle);

	BNO055_Set_ACC_Mode(handle, BNO055_DATA_ACC_RANGE_4G | BNO055_DATA_ACC_BW_62p5Hz |  BNO055_DATA_ACC_MODE_NORMAL);

	BNO055_Get_ACC_Mode(handle);

	BNO055_Set_MAG_Mode(handle, BNO055_DATA_MAG_BW_10Hz | BNO055_DATA_MAG_MODE_HIGHACCURACY | BNO055_DATA_MAG_PWRMODE_NORMAL);

	BNO055_Set_GYR_Mode_1(handle, BNO055_DATA_GYR_RANGE_500DPS | BNO055_DATA_GYR_BW_12Hz);

	BNO055_Set_GYR_Mode_2(handle, BNO055_DATA_GYR_MODE_NORMAL);


	BNO055_Set_OperationMode(handle, BNO055_DATA_OPR_MODE_AMG);

	HAL_Delay(100);

	BNO055_Get_ChipID(handle);
	BNO055_Get_ACC_ID(handle);
	BNO055_Get_GYR_ID(handle);
	BNO055_Get_MAG_ID(handle);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
void BNO055_Reset(BNO055_Sensor_T *handle){
	HAL_GPIO_WritePin(handle->ResetPort, handle->ResetPin, RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(handle->ResetPort, handle->ResetPin, SET);
	HAL_Delay(1000);
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_OperationMode(BNO055_Sensor_T *handle, uint8_t mode){
	uint8_t TxBuff[2], res;
	TxBuff[0] = BNO055_REG_ADDRESS_OPR_MODE;
	TxBuff[1] = mode;

    if(!BNO055_Get_PageID(handle)){
    	if(!handle->PageID){
    		return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0)){
    			return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_OperationMode(BNO055_Sensor_T *handle){
	uint8_t TxBuff[1], RxBuff[1];
	BNO055_ReturnTypeDef_T res= BNO055_ERROR;
	TxBuff[0] = BNO055_REG_ADDRESS_OPR_MODE;

    if(!BNO055_Get_PageID(handle)){
    	if(!handle->PageID){
    		res = BNO055_SendReceive(handle,TxBuff, 1, RxBuff, 1);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0)){
    			res = BNO055_SendReceive(handle,TxBuff, 1, RxBuff, 1);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }

    if(!res)
    	handle->OprMode = RxBuff[0];
    return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_PowerMode(BNO055_Sensor_T *handle, uint8_t mode){
	uint8_t TxBuff[2];
	TxBuff[0] = BNO055_REG_ADDRESS_PWR_MODE;
	TxBuff[1] = mode;

    if(!BNO055_Get_PageID(handle)){
    	if(!handle->PageID){
    		return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0)){
    			return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_ACC_Mode(BNO055_Sensor_T *handle, uint8_t mode){
	uint8_t TxBuff[2];
	TxBuff[0] = BNO055_REG_ADDRESS_ACC_CONFIG;
	TxBuff[1] = mode;

    if(!BNO055_Get_PageID(handle)){
    	if(handle->PageID){
    		return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_1)){
    			return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }
}


/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_ACC_Mode(BNO055_Sensor_T *handle){
	uint8_t TxBuff[1], RxBuff[1];
	TxBuff[0] = BNO055_REG_ADDRESS_ACC_CONFIG;

    if(!BNO055_Get_PageID(handle)){
    	if(handle->PageID){
    		return BNO055_SendReceive(handle,TxBuff, 1, RxBuff, 1);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_1)){
    			return BNO055_SendReceive(handle,TxBuff, 1, RxBuff, 1);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }
}


/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_MAG_Mode(BNO055_Sensor_T *handle, uint8_t mode){
	uint8_t TxBuff[2];
	TxBuff[0] = BNO055_REG_ADDRESS_MAG_CONFIG;
	TxBuff[1] = mode;

    if(!BNO055_Get_PageID(handle)){
    	if(handle->PageID){
    		return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_1)){
    			return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_GYR_Mode_1(BNO055_Sensor_T *handle, uint8_t mode){
	uint8_t TxBuff[2];
	TxBuff[0] = BNO055_REG_ADDRESS_GYR_CONFIG_1;
	TxBuff[1] = mode;

    if(!BNO055_Get_PageID(handle)){
    	if(handle->PageID){
    		return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_1)){
    			return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_GYR_Mode_2(BNO055_Sensor_T *handle, uint8_t mode){
	uint8_t TxBuff[2];
	TxBuff[0] = BNO055_REG_ADDRESS_GYR_CONFIG_2;
	TxBuff[1] = mode;

    if(!BNO055_Get_PageID(handle)){
    	if(handle->PageID){
    		return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_1)){
    			return BNO055_SendReceive(handle,TxBuff, 2, 0, 0);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_ChipID(BNO055_Sensor_T *handle){
	uint8_t Txbuff[1], RxBuff[1];
	BNO055_ReturnTypeDef_T res= BNO055_ERROR;
	Txbuff[0] = BNO055_REG_ADDRESS_CHIP_ID;

    if(!BNO055_Get_PageID(handle)){
    	if(!handle->PageID){
    		res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0)){
    			res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }

    if(!res)
     	handle->IDs.CHIP_ID = RxBuff[0];

     return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_ACC_ID(BNO055_Sensor_T *handle){
	uint8_t Txbuff[1], RxBuff[1];
	BNO055_ReturnTypeDef_T res= BNO055_ERROR;
	Txbuff[0] = BNO055_REG_ADDRESS_ACC_ID;

    if(!BNO055_Get_PageID(handle)){
    	if(!handle->PageID){
    		res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0)){
    			res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }

    if(!res)
     	handle->IDs.ACC_ID = RxBuff[0];

     return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_GYR_ID(BNO055_Sensor_T *handle){
	uint8_t Txbuff[1], RxBuff[1];
	BNO055_ReturnTypeDef_T res= BNO055_ERROR;
	Txbuff[0] = BNO055_REG_ADDRESS_GYR_ID;

    if(!BNO055_Get_PageID(handle)){
    	if(!handle->PageID){
    		res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0)){
    			res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }

    if(!res)
    	handle->IDs.GYR_ID = RxBuff[0];

    return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_MAG_ID(BNO055_Sensor_T *handle){
	uint8_t Txbuff[1], RxBuff[1];
	BNO055_ReturnTypeDef_T res= BNO055_ERROR;
	Txbuff[0] = BNO055_REG_ADDRESS_MAG_ID;

    if(!BNO055_Get_PageID(handle)){
    	if(!handle->PageID){
    		res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    	}else{
    		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0)){
    			res = BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 1);
    		}
    	}
    }else{
    	return BNO055_ERROR;
    }

    if(!res)
        handle->IDs.MAG_ID = RxBuff[0];

    return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_ACC_Data(BNO055_Sensor_T *handle){
	uint8_t Txbuff[1], RxBuff[6];
	BNO055_ReturnTypeDef_T res = BNO055_ERROR;
	Txbuff[0] = BNO055_REG_ADDRESS_ACC_X_DATA_LSB;

	BNO055_Get_PageID(handle);
	if(!handle->PageID)
		res =BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 6);
	else{
		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0))
			res =BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 6);
	}
	if(!res){
		if(!(RxBuff[1]>>7)){
			if( ((RxBuff[1]>>6)&(0x01)) ){
				RxBuff[1] |= 0x80;
			}
		}
		if(!(RxBuff[3]>>7)){
			if( ((RxBuff[3]>>6)&(0x01)) ){
				RxBuff[3] |= 0x80;
			}
		}
		if(!(RxBuff[5]>>7)){
			if( ((RxBuff[5]>>6)&(0x01)) ){
				RxBuff[5] |= 0x80;
			}
		}
		handle->ImuData.ACC_X = (int16_t)((int16_t)(RxBuff[1]<<8) | RxBuff[0]);
		handle->ImuData.ACC_Y = (int16_t)((int16_t)(RxBuff[3]<<8) | RxBuff[2]);
		handle->ImuData.ACC_Z = (int16_t)((int16_t)(RxBuff[5]<<8) | RxBuff[4]);
	}
	return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_MAG_Data(BNO055_Sensor_T *handle){
	uint8_t Txbuff[1], RxBuff[6];
	BNO055_ReturnTypeDef_T res = BNO055_ERROR;
	Txbuff[0] = BNO055_REG_ADDRESS_MAG_X_DATA_LSB;

	BNO055_Get_PageID(handle);
	if(!handle->PageID)
		res =BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 6);
	else{
		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0))
			res =BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 6);
	}

	if(!res){
		handle->ImuData.MAG_X = (RxBuff[0] | (RxBuff[1]<<8));
		handle->ImuData.MAG_Y = (RxBuff[2] | (RxBuff[3]<<8));
		handle->ImuData.MAG_Z = (RxBuff[4] | (RxBuff[5]<<8));
	}
	return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_GYR_Data(BNO055_Sensor_T *handle){
	uint8_t Txbuff[1], RxBuff[6];
	BNO055_ReturnTypeDef_T res = BNO055_ERROR;
	Txbuff[0] = BNO055_REG_ADDRESS_GYR_X_DATA_LSB;

	BNO055_Get_PageID(handle);
	if(!handle->PageID)
		res =BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 6);
	else{
		if(!BNO055_Set_PageID(handle, BNO055_PAGE_ID_0))
			res =BNO055_SendReceive(handle,Txbuff, 1, RxBuff, 6);
		}

	if(!res){
		handle->ImuData.GYR_X = (RxBuff[0] | (RxBuff[1]<<8));
		handle->ImuData.GYR_Y = (RxBuff[2] | (RxBuff[3]<<8));
		handle->ImuData.GYR_Z = (RxBuff[4] | (RxBuff[5]<<8));
	}
		return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_ACCEL_Offset(BNO055_Sensor_T *handle, uint16_t AxisX_Offset, uint16_t AxisY_Offset, uint16_t AxisZ_Offset){
	uint8_t Txbuff[3];
	BNO055_ReturnTypeDef_T res = BNO055_ERROR;

	BNO055_Get_PageID(handle);

	if(handle->PageID){
		BNO055_Set_PageID(handle, BNO055_PAGE_ID_0);
	}
	if(!handle->PageID){
		Txbuff[0] = BNO055_REG_ADDRESS_ACC_X_OFFSET_LSB;
		Txbuff[1] = AxisX_Offset&(0x00FF);
		Txbuff[2] = AxisX_Offset>>8;
		res = BNO055_SendReceive(handle,Txbuff, 3, 0, 0);

		Txbuff[0] = BNO055_REG_ADDRESS_ACC_Y_OFFSET_LSB;
		Txbuff[1] = AxisY_Offset&(0x00FF);
		Txbuff[2] = AxisY_Offset>>8;
		res = BNO055_SendReceive(handle,Txbuff, 3, 0, 0);

		Txbuff[0] = BNO055_REG_ADDRESS_ACC_Z_OFFSET_LSB;
		Txbuff[1] = AxisZ_Offset&(0x00FF);
		Txbuff[2] = AxisZ_Offset>>8;
		res = BNO055_SendReceive(handle,Txbuff, 3, 0, 0);
	}
	return res;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_GetCalibrationState(BNO055_Sensor_T *handle){
	uint8_t TxBuff[1], RxBuff[1];
	TxBuff[0] = 0x35;

	BNO055_Get_PageID(handle);
	if(handle->PageID){
		BNO055_Set_PageID(handle, BNO055_PAGE_ID_0);
	}

	if(!handle->PageID){
		if(!BNO055_SendReceive(handle, TxBuff, 1, RxBuff, 1)){
			handle->CalibState = RxBuff[0];
			return BNO055_OK;
		}
	}
	return BNO055_ERROR;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */

BNO055_ReturnTypeDef_T BNO055_GetCalibrationData(BNO055_Sensor_T *handle){
	uint8_t TxBuff[1], RxBuff[18];
	TxBuff[0] = 0x55;

	BNO055_Get_PageID(handle);
	if(handle->PageID){
		BNO055_Set_PageID(handle, BNO055_PAGE_ID_0);
	}

	if(!handle->PageID){
		if(!BNO055_SendReceive(handle, TxBuff, 1, RxBuff, 18)){
				handle->CalibData.ACC_X = (RxBuff[0] | (RxBuff[1]<<8));
				handle->CalibData.ACC_Y = (RxBuff[2] | (RxBuff[3]<<8));
				handle->CalibData.ACC_Z = (RxBuff[4] | (RxBuff[5]<<8));
				handle->CalibData.MAG_X = (RxBuff[6] | (RxBuff[7]<<8));
				handle->CalibData.MAG_Y = (RxBuff[8] | (RxBuff[9]<<8));
				handle->CalibData.MAG_Z = (RxBuff[10] | (RxBuff[11]<<8));
				handle->CalibData.GYR_X = (RxBuff[12] | (RxBuff[13]<<8));
				handle->CalibData.GYR_Y = (RxBuff[14] | (RxBuff[15]<<8));
				handle->CalibData.GYR_Z = (RxBuff[16] | (RxBuff[17]<<8));
				return BNO055_OK;
		}
	}
	return BNO055_ERROR;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_DataUnit(BNO055_Sensor_T *handle, uint8_t data){
	uint8_t TxBuff[2];
	TxBuff[0] = 0x3B;
	TxBuff[1] = data;

	BNO055_Get_PageID(handle);
	if(handle->PageID){
		BNO055_Set_PageID(handle, BNO055_PAGE_ID_0);
	}

	if(!handle->PageID){
		return BNO055_SendReceive(handle, TxBuff, 2, 0, 0);
	}
	return BNO055_ERROR;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_DataUnit(BNO055_Sensor_T *handle){
	uint8_t TxBuff[1], RxBuff[1];
	TxBuff[0] = 0x3B;

	BNO055_Get_PageID(handle);
	if(handle->PageID){
		BNO055_Set_PageID(handle, BNO055_PAGE_ID_0);
	}
	if(!handle->PageID){
		if(!BNO055_SendReceive(handle, TxBuff, 1, RxBuff, 1)){
			handle->UnitStat = RxBuff[0];
			return BNO055_OK;
		}
	}
	return BNO055_ERROR;

}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_PageID(BNO055_Sensor_T *handle){
	uint8_t TxBuff[1], RxBuff[1];
	TxBuff[0] = 0x07;
	if(BNO055_SendReceive(handle, TxBuff, 1, RxBuff, 1))
		return BNO055_ERROR;
	else
		handle->PageID = RxBuff[0];
	return BNO055_OK;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Set_PageID(BNO055_Sensor_T *handle, uint8_t PageID){
	uint8_t TxBuff[2];
	TxBuff[0] = 0x07;
	TxBuff[1] = PageID;
	if(BNO055_SendReceive(handle, TxBuff, 2, 0, 0))
		return BNO055_ERROR;
	else{
		HAL_Delay(5);
		BNO055_Get_PageID(handle);
		return BNO055_OK;
	}
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_Get_SysError(BNO055_Sensor_T *handle){
	uint8_t TxBuff[1], RxBuff[1];
	TxBuff[0] = 0x3A;
	if(BNO055_SendReceive(handle, TxBuff, 1, RxBuff, 1))
		return BNO055_ERROR;
	else
		handle->SysError = RxBuff[0];
	return BNO055_OK;
}

/** Brief description which ends at this dot. Details follow
 *  here.
 */
BNO055_ReturnTypeDef_T BNO055_SendReceive(BNO055_Sensor_T *handle, uint8_t *txBuff, uint8_t txLenght, uint8_t *rxBuff, uint8_t rxLenght){
	return BNO055_CommPorter_SendReceive(handle->I2C_No, handle->Chip_I2C_Address, txBuff, txLenght, rxBuff, rxLenght);
}
