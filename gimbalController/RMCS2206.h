/*
RMCS2206.h - Library for using the L-3 Communications NimbleGimble Prototype

Created by Team Align (#16 Capstone 2015-2016)
Use of this code without written permission is strictly prohibited.

*/

#ifndef RMCS2206_h
#define RMCS2206_h

class RMCS2206
{
	public:
	
		RMCS2206(char dof, int serialNumber);
		int begin();
		void set_position(float deg);
		void set_speed(float degPerSec);
		void move_relative(float deg);
		
		
	private:
		
		bool _faultFlag = 0;
		int _serialNumber;
		float _degPerEnc;
		int _dof;
	
};

#endif