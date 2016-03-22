/*
FPR_control.h - Library for using the L-3 Communications NimbleGimble Prototype

Created by Team Align (#16 Capstone 2015-2016)
Use of this code without written permission is strictly prohibited.

*/

#ifndef FPR_control_h
#define FPR_control_h

#include <Arduino.h>

class FPR_control
{
	
	public:
	
		FPR_control();
		void begin();
		void set_gains(float kp, float ki, float kd);
		void set_pins(int pwmPin, int dirPin);
		void set_precision(int pwmBits, int adcBits);
		void set_antiwindup(float intThreshHigh, float intThreshLow);
		void set_time_filter(float Ts, float alpha, float tau);
		float pid(float state, float command, bool flag);
		void dir(float control);
		void pwm(float control);
		void stop();
		
		/*void add_time(unsigned long);
		void update_gyroPos(float xPos, float yPos, float zPos);
		bool phase_found(char mode);
		float follow_command(char mode);*/


		
	private:
		
		//Functions
		float saturate(float input, float highLimit, float lowLimit);
		
		//Variables
		float _integrator, _differentiator, _error_d1, _intThreshHigh, _intThreshLow, _tau, _alpha, _Ts;
		float _kp, _ki, _kd;

		//Constants
		float _contThresh = 1.0;
		float _tableAlpha = .05;
		
		//Pins
		int _pwmPin, _dirPin, _pwmBits, _adcBits;
		
		//Other Algorithm Requirements
		int _minNumber;
		float _tZero[3];
		float _time;
		float _phaseShift;
		float _searchPos[2];
		float _tablePos;

		float _gyroPos[3];
};

#endif