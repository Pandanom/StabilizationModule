/*
 * PID.cpp
 *
 * Created: 21.05.2021 1:39:16
 *  Author: Saas
 */ 

#include "PID.h"
#include <Arduino.h>
//double ki, kd, kp;
//unsigned long preTime;
//double dT, preErr;

PID::PID(double min_, double max_){
	ki = 1;
	kd = 1;
	kp = 1;
	min = min_;
	max = max_;
	preTime = millis();
}

void PID::setK(double kp_, double ki_, double kd_){
		ki = ki_;
		kd = kd_;
		kp = kp_;
}

void PID::setPos(double pos_){
	pos = pos_;
}

double PID::calcReg(double inp_){
	//calculate time interval
	auto currTime = millis();
	double dT = (double)(preTime - currTime);
	preTime = currTime;
	
	//get error
	double error = pos - inp_;
	
	//calculate every part of PID
	double pOut = kp * error;
	double iOut = ki * error * dT;
	double dOut = kd * ((error - preErr) / dT);
	
	//calculate total output
	double output = pOut + iOut + dOut;
	
	//restrict to max/min
	if( output > max )
		output = max;
	else if( output < min )
		output = min;
		
	//save this error for next iteration
	preErr = error;
	
	return output;
}



