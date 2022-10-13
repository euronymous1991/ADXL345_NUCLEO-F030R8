/*
 *
 * ADK345 Accelerometer I2C driver
 *
 */
 
 #ifndef ADXL345_I2C_DRIVER_H
 #define ADXL345_I2C_DRIVER_H
 
 #include "stm32f0xx_hal.h" /* Needed for I2C */
 
 
 /*
 * DEFINES
 */
 #define ADXL345_i2C_ADDR (0x53<<1) /* ASEL = 0 -> 0x1D, ASEL = -> 0x53 p 19*/
 
 #define ADXL345_DEVICE_ID 0xE5
 
 /*
  * REGISTERS (p. 24)
	*/
	//      Register Name 												 Description 																								Reset value	 	Type			
	#define ADXL345_REG_DEVID 						0x00 /*  Device ID, 																									11100101,  R 		*/
	#define ADXL345_REG_THESH_TAP 				0x1D /*  Tap threshold, 																						  00000000,  R/W  */
	#define ADXL345_REG_OFSX 							0x1E /*  X-axis offset, 																						  00000000,  R/W  */
	#define ADXL345_REG_OFSY 							0x1F /*  Y-axis offset, 																						  00000000,  R/W  */
	#define ADXL345_REG_OFSZ							0x20 /*  Z-axis offset,																							  00000000,  R/W  */
	#define ADXL345_REG_DUR 							0x21 /*  Tap Duration, 																							  00000000,  R/W  */
	#define ADXL345_REG_Latent 						0x22 /*  Tap latency, 																							  00000000,  R/W  */
	#define ADXL345_REG_Window 						0x23 /*  Tap window, 																								  00000000,  R/W  */
	#define ADXL345_REG_THRESH_ACT				0x24 /*  Activity threshold, 																				  00000000,  R/W  */
	#define ADXL345_REG_THRESH_INACT			0x25 /*  Inactivity threshold, 																			  00000000,  R/W  */
	#define ADXL345_REG_TIME_INACT				0x26 /*  Inactivity time, 																					  00000000,  R/W  */
	#define ADXL345_REG_ACT_INACT_CTL 		0x27 /*  Axis enable control for activity and inactivity detection,   00000000,  R/W  */
	#define ADXL345_REG_THRESH_FF					0x28 /*  Free-fall threshold,																				  00000000,  R/W  */
	#define ADXL345_REG_TIME_FF 					0x29 /*  Free-fall time, 																						  00000000,  R/W  */
	#define ADXL345_REG_TAP_AXES 					0x2A /*  Axis control for single tap/double tap, 										  00000000,  R/W  */
	#define ADXL345_REG_ACT_TAP_STATUS		0x2B /*  Source of single tap/double tap, 													  00000000,  R  	*/
	#define ADXL345_REG_BW_RATE  					0x2C /*  Data rate and power mode control,									 				  00001010,  R/W  */
	#define ADXL345_REG_POWER_CTL			 		0x2D /*  Power-saving features control, 														  00000000,  R/W  */
	#define ADXL345_REG_INT_ENABLE 		 		0x2E /*  Interrupt enable control,			 														  00000000,  R/W  */
	#define ADXL345_REG_INT_MAP				 		0x2F /*  Interrupt mapping control, 																  00000000,  R/W  */	
	#define ADXL345_REG_INT_SOURCE 		 		0x30 /*  Source of interrupts, 																			  00000000,  R 	 	*/
	#define ADXL345_REG_DATA_FORMAT		 		0x31 /*  Data format control, 																			  00000000,  R/W  */	
	#define ADXL345_REG_DATAX0 				 		0x32 /*  X-Axis Data 0, 																						  00000000,  R  	*/	
	#define ADXL345_REG_DATAX1				 		0x33 /*  X-Axis Data 1, 																						  00000000,  R  	*/	
 	#define ADXL345_REG_DATAY0				 		0x34 /*  Y-Axis Data 0, 																						  00000000,  R  	*/
	#define ADXL345_REG_DATAY1				 		0x35 /*  Y-Axis Data 1, 																						  00000000,  R  	*/
	#define ADXL345_REG_DATAZ0				 		0x36 /*  Z-Axis Data 0, 																						  00000000,  R  	*/
	#define ADXL345_REG_DATAZ1				 		0x37 /*  Z-Axis Data 1, 																						  00000000,  R  	*/
	#define ADXL345_REG_FIFO_CTL			 		0x38 /*  FIFO control, 																							  00000000,  R/W  */
	#define ADXL345_REG_FIFO_STATUS 	 		0x39 /*  FIFO status , 																							  00000000,  R  	*/

 /* 
	*SENSOR STRUCT
	*/

	typedef struct {
		/*I2C Handle */
		I2C_HandleTypeDef *i2cHandle;
		
		/* Acceleration data (X, Y, Z) in m/s^2 */
		float acc_mps2[3];
		
	} ADXL345;
	
	/*
	* INITIALISATION
	*/
	uint8_t ADXL345_Initialise( ADXL345 *dev, I2C_HandleTypeDef *i2cHandle);
	
	/*
	* DATA AQUISITION
	*/
	
	HAL_StatusTypeDef	ADXL345_ReadAccelerations(ADXL345 *dev);
	
	/*
	 * LOW LEVEL FUNCTIONS
	 */
  HAL_StatusTypeDef ADXL345_ReadRegister( ADXL345 *dev, uint8_t reg, uint8_t *data);
	HAL_StatusTypeDef ADXL345_ReadRegisters( ADXL345 *dev, uint8_t reg, uint8_t *data, uint8_t length);
	
	HAL_StatusTypeDef ADXL345_WriteRegister( ADXL345 *dev, uint8_t reg, uint8_t *data);
	
	
 #endif
 
 
 
 
 