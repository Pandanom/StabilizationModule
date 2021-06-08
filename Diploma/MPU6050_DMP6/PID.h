/*
 * PID.h
 *
 * Created: 21.05.2021 0:53:48
 *  Author: Saas
 */ 


#ifndef PID_H_
#define PID_H_


class PID{
//coefficients
double ki, kd, kp;
//previous time to calculate dt	
unsigned long preTime;
//previous error
double preErr;
//needed value
double pos;
//minimum and maximum output value
double min, max;


public:
//constructor
PID(double min_, double max_);

//set coefficients
void setK(double kp_, double ki_, double kd_);

//set needed position
void setPos(double pos_);

//calculate output
double calcReg(double inp_);

};




#endif /* PID_H_ */