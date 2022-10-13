
//#include "stm32f0xx_hal.h" /* Needed for I2C */
#include "ADXL345.h"

uint8_t ADXL345_Initialise( ADXL345 *dev, I2C_HandleTypeDef *i2cHandle){
	/* Set struct parameters */
	dev-> i2cHandle = i2cHandle;
	
	dev->acc_mps2[0] 	=0.0f;
	dev->acc_mps2[1] 	=0.0f;
	dev->acc_mps2[2] 	=0.0f;
	
	/* Klaidu skaicius  grazinamas funkcijos pabaigoje */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	
	/* device id patikrinimas (DATASHEET p. */
	uint8_t regData;
	
	status = ADXL345_ReadRegister(dev, ADXL345_REG_DEVID, &regData);
	errNum +=(status !=HAL_OK);
	
	if(regData !=ADXL345_DEVICE_ID) {
		
		return 255;
	}
	
	//paleidimas
	regData = 0x00;
	
	status = ADXL345_WriteRegister( dev, ADXL345_REG_POWER_CTL, &regData);
	errNum +=( status !=HAL_OK);
		
	regData = 0x08;
	
	status = ADXL345_WriteRegister( dev, ADXL345_REG_POWER_CTL, &regData);
	errNum +=( status !=HAL_OK);
	
	regData = 0x0A;
	
	status = ADXL345_WriteRegister( dev, ADXL345_REG_BW_RATE, &regData);
	errNum +=( status !=HAL_OK);
	
	regData = 0x80;
	
	status = ADXL345_WriteRegister( dev, ADXL345_REG_INT_ENABLE, &regData);
	errNum +=( status !=HAL_OK);
	
	regData = 0x01;
	
	status = ADXL345_WriteRegister( dev, ADXL345_REG_DATA_FORMAT, &regData);
	errNum +=( status !=HAL_OK);
	

	
	return errNum;
}

HAL_StatusTypeDef	ADXL345_ReadAccelerations(ADXL345 *dev){
	/* Datasheet pages */
	/* 
	* Nuskaityti raw duomenis x, y, z, 16bits
	*/
		uint8_t regData[6];
		HAL_StatusTypeDef status = ADXL345_ReadRegisters(dev, ADXL345_REG_DATAX0, regData, 6);
	
		int16_t accRawSigned[3];
		accRawSigned[0]=((regData[1]<<8)|regData[0]);  
		accRawSigned[1]=((regData[3]<<8)|regData[2]); 
		accRawSigned[2]=((regData[5]<<8)|regData[4]); 
	
	/* Konvertavimas */
			dev->acc_mps2[0] = (accRawSigned[0]*0.0072);
			dev->acc_mps2[1] = (accRawSigned[1]*0.0072);
			dev->acc_mps2[2] = (accRawSigned[2]*0.0072);
	
	return status;
}

/* 
 * LOW-LEVEL FUNCTIONS
 */
 
   HAL_StatusTypeDef ADXL345_ReadRegister( ADXL345 *dev, uint8_t reg, uint8_t *data)
		 {
		 
		 return HAL_I2C_Mem_Read (dev->i2cHandle, ADXL345_i2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
		 
		 }
	HAL_StatusTypeDef ADXL345_ReadRegisters( ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t length)
		 {
			 
		 return HAL_I2C_Mem_Read (dev->i2cHandle, ADXL345_i2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
		 
		 }
		 	
	HAL_StatusTypeDef ADXL345_WriteRegister( ADXL345 *dev, uint8_t reg, uint8_t *data)
		{		
			
			return HAL_I2C_Mem_Write (dev->i2cHandle, ADXL345_i2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
			
		}


		