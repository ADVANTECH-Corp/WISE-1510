#ifndef __BMA680_H__
#define __BMA680_H__

#ifdef __cplusplus
extern "C"
{
#endif

//#include <ti/drivers/I2C.h>
//#include "common/common.h"

// TI HDC1050
//	1. Configure the acquisition parameters in register address 0x02:
//	(a) Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1.
//	(b) Set the desired temperature measurement resolution:
//  	¡V Set Bit[10] to 0 for 14 bit resolution.
// 		¡V Set Bit[10] to 1 for 11 bit resolution.
// (c) Set the desired humidity measurement resolution:
// 		¡V Set Bit[9:8] to 00 for 14 bit resolution.
//		¡V Set Bit[9:8] to 01 for 11 bit resolution.
// 		¡V Set Bit[9:8] to 10 for 8 bit resolution.
// 	2. Trigger the measurements by executing a pointer write transaction with the address pointer set to 0x00.
// 		Refer to Figure 12.
// 3. Wait for the measurements to complete, based on the conversion time (refer to Electrical Characteristics(1)
// 		for the conversion time).

///	Register Map
//	0x00 Temperature 0x0000 Temperature measurement output
//	0x01 Humidity 0x0000 Relative Humidity measurement output
//	0x02 Configuration 0x1000 HDC1050 configuration and status
//	0xFB Serial ID device dependent First 2 bytes of the serial ID of the part
//	0xFC Serial ID device dependent Mid 2 bytes of the serial ID of the part
//	0xFD Serial ID device dependent Last byte bit of the serial ID of the part
//	0xFE Manufacturer ID 0x5449 ID of Texas Instruments
//	0xFF Device ID 0x1050 ID of HDC1050 device

#define BOSCH_BMA680_DEVICE_ADDR			0X33

#define BOSCH_BMA680_DEVICE_ID 				0x00
#define BOSCH_BMA680_TEMPERATURE_ADDR		0X01
#define BOSCH_BMA680_PRESSURE_ADDR			0X03
#define BOSCH_BMA680_HUMIDITY_ADDR			0X05
#define BOSCH_BMA680_IAQ_ADDR				0X07
#define BOSCH_BMA680_IAQ_ACCURACY_ADDR		0X09

//void 	BMA680_Init(void);
//void 	BMA680_UnInit(void);
//unsigned int void BMA680_GetSensorData(float *Temperature, float *Pressure, float *Humidity, uint16_t *IAQ, uint8_t *Accuracy, uint8_t *raw);


#ifdef __cplusplus
}
#endif


#endif // End of __BMA680_H__
