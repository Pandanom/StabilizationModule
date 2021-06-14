/* 
* GimbalMot.cpp
*
* Created: 21.05.2021 16:09:47
* Author: Saas
*/


#include "GimbalMot.h"
#include <avr/io.h>
#include <Arduino.h>

//Motor* mX, mY, mZ;

// default constructor
GimbalMot::GimbalMot()
{
} //GimbalMot

// default destructor
GimbalMot::~GimbalMot()
{
	delete mX;
	delete mY;
	delete mZ;
} //~GimbalMot

//initialize pins 
void GimbalMot::init(int aX_, int bX_, int aY_, int bY_, int aZ_, int bZ_){
	//turn up frequency to reduce noise
	//TCCR1B = (TCCR1B & 0xf8) | 1; // 31 371 Hz
	//TCCR2B = (TCCR2B & 0xf8) | 1; // 31 371 Hz
	TCCR1B = 0;
	TCCR2B = 0;
	TCCR1B |= (1 << CS10);
	TCCR1B |= (1 << CS12);
	TCCR2B |= (1 << CS10);
	TCCR2B |= (1 << CS12);
	// set pin mode output for every pin
	pinMode(aX_, OUTPUT);
	pinMode(bX_, OUTPUT);
	pinMode(aY_, OUTPUT);
	pinMode(bY_, OUTPUT);
	pinMode(aZ_, OUTPUT);
	pinMode(bZ_, OUTPUT);
	//create motor for every axis 
	mX = new Motor(aX_, bX_);
	mY = new Motor(aY_, bY_);
	mZ = new Motor(aZ_, bZ_);
}

//set velocity for x axis
void GimbalMot::setVelX(int x_){
	mX->setO(x_);
}

//set velocity for y axis
void GimbalMot::setVelY(int y_){
	mY->setO(y_);
}

//set velocity for z axis
void GimbalMot::setVelZ(int z_){
	mZ->setO(z_);
}


//int v;
//int direction;
//int outA, outB;

Motor::Motor(int outA_, int outB_){
	outA = outA_;
	outB = outB_;
	v = 0;
	direction = false;
}

//set output
void Motor::setO(int v_){
	v_ = constrain(v_, -255, 255);
	if( v_ < 0)
	{
		v = -v_;
		direction = false;
	}
	else
	{
		v = v_;
		direction = true;
	}
	setPWM();
}

void Motor::setPWM(){
	if(direction)
	{
	  analogWrite(outA, 0);
	  analogWrite(outB, v);
	}
	else
	{
	  analogWrite(outA, v);
	  analogWrite(outB, 0);
	}
}