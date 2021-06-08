/* 
* GimbalMot.h
*
* Created: 21.05.2021 16:09:47
* Author: Saas
*/

#ifndef __GIMBALMOT_H__
#define __GIMBALMOT_H__

class Motor
{
	int v;
	int direction;
	int outA, outB;
	
	//set pwm value
	void setPWM();
	
	public:
	//set output pins
	Motor(int outA_, int outB_);
	
	//set output
	void setO(int v_);
};

class GimbalMot
{

//variables
Motor *mX, *mY, *mZ;

//functions
public:
	//ctor
	GimbalMot();
	//dtor
	~GimbalMot();
	
	//initialize pins 
	void init(int aX_, int bX_, int aY_, int bY_, int aZ_, int bZ_);
	
	//set velocity for x axis
	void setVelX(int x_);

	//set velocity for y axis
	void setVelY(int y_);

	//set velocity for z axis
	void setVelZ(int z_);
}; //GimbalMot



#endif //__GIMBALMOT_H__
