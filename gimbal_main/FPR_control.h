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
	
		FPR_control(void);
		void begin();
		void set_gains(float kp, float ki, float kd);
		void set_pins(int pwmPin, int dirPin);
		void set_precision(int pwmBits, int adcBits);
		void set_antiwindup(float intThreshHigh, float intThreshLow);
		void set_time_filter(float Ts, float alpha, float tau);
		float pid(float state, float command, bool flag);
		
		
	private:
		
		//Functions
		float saturate(float input, float highLimit, float lowLimit);
		
		//Variables
		float _integrator, _differentiator, _error_d1, _intThreshHigh, _intThreshLow, _tau, _alpha, _Ts;
		float _kp, _ki, _kd;
				
		float _contThresh = 1.0;
		int _pwmPin, _dirPin, _pwmBits, _adcBits;
		
};

#endif