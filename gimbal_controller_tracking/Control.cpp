#include <Arduino.h>
#include "Control.h"
#include <math.h>

Control::Control(void)
{
	
	
	
}

void Control::begin()
{
	
	
	
}

void Control::set_time_filter(float Ts, float alpha, float tau, char type)
{
	if(type == 'p')
	{
		_Ts_pos = Ts;
		_alpha_pos = alpha;
		_tau_pos = tau;
	}
	else if(type == 'v')
	{
		_Ts_vel = Ts;
		_alpha_vel = alpha;
		_tau_vel = tau;
	}
	else
	{
		while(1);
	}
}

void Control::set_gains(float kp, float ki, float kd, char type)
{
	if(type == 'p')
	{
		_kp_pos = kp;
		_ki_pos = ki;
		_kd_pos = kd;	
	}
	else if(type == 'v')
	{
		_kp_vel = kp;
		_ki_vel = ki;
		_kd_vel = kd;
	}
	else
	{
		while(1);
	}
}

void Control::set_precision(int pwmBits, int adcBits)
{
	_pwmBits = pwmBits;
	_adcBits = adcBits;
	
	//ADC and PWM precision
	analogWriteResolution(pwmBits);
	analogReadResolution(adcBits);
	
}

void Control::set_antiwindup(float intThreshHigh, float intThreshLow, char type)
{
	if(type == 'p')
	{
		_intThreshHigh_pos = intThreshHigh;
		_intThreshLow_pos = intThreshLow;
	}
	else if(type == 'v')
	{
		_intThreshHigh_vel = intThreshHigh;
		_intThreshLow_vel = intThreshLow;
	}
	else
	{
		while(1);
	}
}

float Control::position(float state, float command, bool flag)
{
	
	if(flag)
	{
		_integrator_pos = 0.0;
		_differentiator_pos = 0.0;
		_error_d1_pos = 0.0;
	}
	
	float error = command - state;
	//Serial.println(error);
	
	if(abs(error) < _intThreshHigh_pos && abs(error) > _intThreshLow_pos)
	{
		_integrator_pos = _integrator_pos + (_Ts_pos/2.0)*(error + _error_d1_pos);
	}
	else
	{
		_integrator_pos = 0.0;
	}
	
	_differentiator_pos = ((2.0*_tau_pos - _Ts_pos)/(2.0*_tau_pos + _Ts_pos))*_differentiator_pos + (2.0/(2.0*_tau_pos + _Ts_pos))*(error - _error_d1_pos);
	
	_error_d1_pos = error;
	
	//Serial.print(_kp);Serial.print("\t");Serial.print(_ki);Serial.print("\t");Serial.println(_kd);
	
	//Serial.println(saturate(_kp*error + _ki*_integrator + _kd*_differentiator, _contThresh, 0.0));
	
	//return(_kp*error + _ki*_integrator + _kd*_differentiator);
	return(saturate((_kp_pos*error + _ki_pos*_integrator_pos + _kd_pos*_differentiator_pos), 1.0, -1.0));

}


float Control::velocity(float state, float command, bool flag)
{
	
	if(flag)
	{
		_integrator_vel = 0.0;
		_differentiator_vel = 0.0;
		_error_d1_vel = 0.0;
	}
	
	float error = command - state;
	//Serial.println(error);
	
	if(abs(error) < _intThreshHigh_vel && abs(error) > _intThreshLow_vel)
	{
		_integrator_vel = _integrator_vel + (_Ts_vel/2.0)*(error + _error_d1_vel);
	}
	else
	{
		_integrator_vel = 0.0;
	}
	
	_differentiator_vel = ((2.0*_tau_vel - _Ts_vel)/(2.0*_tau_vel + _Ts_vel))*_differentiator_vel + (2.0/(2.0*_tau_vel + _Ts_vel))*(error - _error_d1_vel);
	
	_error_d1_vel = error;
	
	//Serial.print(_kp);Serial.print("\t");Serial.print(_ki);Serial.print("\t");Serial.println(_kd);
	
	//Serial.println(saturate(_kp*error + _ki*_integrator + _kd*_differentiator, _contThresh, 0.0));
	
	//return(_kp*error + _ki*_integrator + _kd*_differentiator);
	return(saturate((_kp_vel*error + _ki_vel*_integrator_vel + _kd_vel*_differentiator_vel), 1.0, -1.0));

}


void Control::set_pins(int pwmPin, int dirPin)
{
	_pwmPin = pwmPin;
	_dirPin = dirPin;
	
	pinMode(_pwmPin, OUTPUT);
	pinMode(_dirPin, OUTPUT);
}


float Control::saturate(float input, float highLimit, float lowLimit)
{
	float output;
	
	if(input >= highLimit)
	{
		output = highLimit;
		//Serial.println("High Limit");
	}
	else if(input < lowLimit)
	{
		output = lowLimit;
		//Serial.println("Low Limit");
	}
	else
	{
		output = input;
		//Serial.println("Good");
	}
	
	//Serial.println(output);
	return(output);
}


float Control::x_rate_filter(float signal, float alpha)
{
	static float oldSignal, newSignal;
	
	newSignal = oldSignal*(1-alpha)+signal*alpha;
	oldSignal = newSignal;
	return(newSignal);
}
float Control::y_rate_filter(float signal, float alpha)
{
	static float oldSignal, newSignal;
	
	newSignal = oldSignal*(1-alpha)+signal*alpha;
	oldSignal = newSignal;
	return(newSignal);
}
float Control::z_rate_filter(float signal, float alpha)
{
	static float oldSignal, newSignal;
	
	newSignal = oldSignal*(1-alpha)+signal*alpha;
	oldSignal = newSignal;
	return(newSignal);
}



