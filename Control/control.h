/*
 * control.h
 *
 */

#ifndef CONTROL_H_
#define CONTROL_H_

typedef struct
{
	double value;
	double min;
	double max;
	double gain;
}ControlValueType;

typedef struct
{
	double error;
	double last_error;
	ControlValueType p;
	ControlValueType i;
	double d;
	double out;

} ControlStructType;

void PID_conrol (ControlStructType* ControlStruct);


#endif /* CONTROL_H_ */
