/*
Control.h - Library for using the L-3 Communications NimbleGimble Prototype

Created by Team Align (#16 Capstone 2015-2016)
Use of this code without written permission is strictly prohibited.

*/

#ifndef Control_h
#define Control_h

#include <Arduino.h>

class Control
{
	
	public:
	
		Control(void);
		void begin();
		void set_gains(float kp, float ki, float kd, char type);
		void set_pins(int pwmPin, int dirPin);
		void set_precision(int pwmBits, int adcBits);
		void set_antiwindup(float intThreshHigh, float intThreshLow, char type);
		void set_time_filter(float Ts, float alpha, float tau, char type);
		float position(float state, float command, bool flag);
		float velocity(float state, float command, bool flag);
		float x_rate_filter(float signal, float alpha);
		float y_rate_filter(float signal, float alpha);
		float z_rate_filter(float signal, float alpha);
		
		
	private:
		
		//Functions
		float saturate(float input, float highLimit, float lowLimit);
		
		//Variables
		float _integrator_pos, _differentiator_pos, _error_d1_pos, _intThreshHigh_pos, _intThreshLow_pos, _tau_pos, _alpha_pos, _Ts_pos;
		float _kp_pos, _ki_pos, _kd_pos;
		
		float _integrator_vel, _differentiator_vel, _error_d1_vel, _intThreshHigh_vel, _intThreshLow_vel, _tau_vel, _alpha_vel, _Ts_vel;
		float _kp_vel, _ki_vel, _kd_vel;
		
		float _contThresh = 1.0;
		int _pwmPin, _dirPin, _pwmBits, _adcBits;
		
};

#endif