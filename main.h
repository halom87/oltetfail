/*
 * main.h
 *
 *  Created on: 2012.10.21.
 *      Author: Adam
 */

#ifndef MAIN_H_
#define MAIN_H_

//void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName );
typedef struct
{
	float AXIS_X;
	float AXIS_Y;
	float AXIS_Z;

} AxesData;
typedef struct
{
	AxesData Acc;
	AxesData Mag;
	AxesData Gyro;

} SensorData_t;


#endif /* MAIN_H_ */
