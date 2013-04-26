/*
 * control.c
 *
 */
#include "control.h"
void PID_conrol (ControlStructType* ControlStruct)
{
	if ((ControlStruct->i.value = ControlStruct->i.value+ControlStruct->error*ControlStruct->i.gain)>ControlStruct->i.max) ControlStruct->i.value=ControlStruct->i.max;
	else if (ControlStruct->i.value<ControlStruct->i.min) ControlStruct->i.value=ControlStruct->i.min;
	ControlStruct->p.value= (ControlStruct->error-ControlStruct->last_error)*ControlStruct->d;
	ControlStruct->last_error=ControlStruct->error;
	if ((ControlStruct->p.value += ControlStruct->i.value+ControlStruct->error*ControlStruct->p.gain)>ControlStruct->p.max) ControlStruct->p.value=ControlStruct->p.max;
	else if (ControlStruct->p.value<ControlStruct->p.min) ControlStruct->p.value=ControlStruct->p.min;
	ControlStruct->out=ControlStruct->p.value;

}
