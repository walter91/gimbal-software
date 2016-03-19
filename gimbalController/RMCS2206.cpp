#include <Arduino.h>
#include "RMCS2206.h"

RMCS2206::RMCS2206(char dof, int serialNumber)
{
	/*
	Initialize an RMCS2206 object as either 'A' for azimuth
	or 'E' for elevation. Calculations for velocity, acceleration
	and position will be based on this characteristic.
	*/
	
	_dof = dof;
	_serialNumber = serialNumber;
	
	if(_dof == 'A')
		_degPerEnc = 0.2;
	else if(_dof == 'E')
		_degPerEnc = 0.2;
	else
		_faultFlag = 1;
	
//	return(_faultFlag);
	
}

int RMCS2206::begin()
{
	/*
	Initialize serial communication.
	Then set all parameters for the motor via serial communication.
	'M' for max motor speed (0 - 255)
	'B' for P gain term (0 - 32767)
	'C' for I gian term (0 - 32767)
	'D' read/write speed dampening (0 - 255)
	'P' read/write encoder position (0 - 2147483647)
	*/
	
	switch(_serialNumber)
	{
		case 1:
			#define serial Serial1
			break;
		case 2:
			#define serial Serial2
			break;
		case 3:
			#define serial Serial3
			break;
		default:
			_faultFlag = 1;
			break;
	}
	
	serial.begin(9600);
	
	
	return(_faultFlag);
	
}

void RMCS2206::set_position(float deg)
{
	//
	
	
	
}

void RMCS2206::set_speed(float degPerSec)
{
	//
	
	
	
}

void RMCS2206::move_relative(float deg)
{
	//Move a relative encoder count using 'R' command
	
	if(deg > 0)
	{
		serial.print("R"); serial.print("+"); serial.println(int(abs(deg)/_degPerEnc));
	}
	else if(deg < 0)
	{
		serial.print("R"); serial.print("+"); serial.println(int(abs(deg)/_degPerEnc));
	}
	else
	{
		serial.print("R"); serial.print(0);
	}
	
	Serial.print("Should be moving to a relative position of: "); Serial.println(deg);
}
/* 
int RMCS2206::read_encoder()
{
	
	serial.
	
} */





