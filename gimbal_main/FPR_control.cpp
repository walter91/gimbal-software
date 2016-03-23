#include "FPR_control.h"

#include <Arduino.h>
#include <math.h>

#define SCALE 5
#define OMEGA 2

FPR_control::FPR_control(void)
{
	//nothing yet
}

void FPR_control::begin()
{
	_minNumber = 0;
}

void FPR_control::set_time_filter(float Ts, float alpha, float tau)
{
	_Ts = Ts;
	_alpha = alpha;
	_tau = tau;
	
}

void FPR_control::set_gains(float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;	

}

void FPR_control::set_precision(int pwmBits, int adcBits)
{
	_pwmBits = pwmBits;
	_adcBits = adcBits;
	
	//ADC and PWM precision
	analogWriteResolution(_pwmBits);
	analogReadResolution(_adcBits);
	
}

void FPR_control::set_antiwindup(float intThreshHigh, float intThreshLow)
{
	_intThreshHigh = intThreshHigh;
	_intThreshLow = intThreshLow;
}

float FPR_control::pid(float state, float command, bool flag)
{
	
	if(flag)
	{
		_integrator = 0.0;
		_differentiator = 0.0;
		_error_d1 = 0.0;
	}
	
	float error = command - state;
	
	if(abs(error) < _intThreshHigh && abs(error) > _intThreshLow)
	{
		_integrator = _integrator + (_Ts/2.0)*(error + _error_d1);
	}
	else
	{
		_integrator = 0.0;
	}
	
	_differentiator = ((2.0*_tau - _Ts)/(2.0*_tau + _Ts))*_differentiator + (2.0/(2.0*_tau + _Ts))*(error - _error_d1);
	
	_error_d1 = error;
	
	return(saturate((_kp*error + _ki*_integrator + _kd*_differentiator), 1.0, -1.0));

}


void FPR_control::set_pins(int pwmPin, int dirPin)
{
	_pwmPin = pwmPin;
	_dirPin = dirPin;
	
	pinMode(_pwmPin, OUTPUT);
	pinMode(_dirPin, OUTPUT);
}


float FPR_control::saturate(float input, float highLimit, float lowLimit)
{
	float output;
	
	if(input >= highLimit)
	{
		output = highLimit;
	}
	else if(input < lowLimit)
	{
		output = lowLimit;
	}
	else
	{
		output = input;
	}
	
	return(output);
}


void FPR_control::dir(float control)
{
	bool dir;

	if(control >= 0)
	{
		dir = 0;
	}
	else
	{
		dir = 1;
	}
	
	digitalWrite(_dirPin, dir);
}


void FPR_control::pwm(float control)
{
	float controlEffort = 4095.0*abs(control);

	//controlEffort = saturate(controlEffort, 4095.0 , 0.0);

	Serial.println(controlEffort);

	analogWrite(_pwmPin, controlEffort);
}


void FPR_control::stop()
{
	analogWrite(_pwmPin, 0);
	
}

/*
bool FPR_control::phase_found(char mode)
{
	bool flag = false;
	float _gyroPosition;

	if(mode == 'p')
	{
		_gyroPosition = _gyroPos[0];
	}
	else if(mode == 'y')
	{
		_gyroPosition = _gyroPos[1];
	}
	else //mode == "r"
	{
		_gyroPosition = _gyroPos[2];
	}


	switch(_minNumber)
	{
		case(0):
			//fill _searchPos[1]
			//if _searchPos[0]*_searchPos[1]<0
				//tZero[0] = t
				//minNumber = minNumber+1
			//else
				//do nothing (min not found)
			break;
		case(1):
			//fill _searchPos[1]
			//if _searchPos[0]*_searchPos[1]<0
				//tZero[1] = t
				//minNumber = minNumber+1
			//else
				//do nothing (min not found)
			break;
		case(2):
			//fill _searchPos[1]
			//if _searchPos[0]*_searchPos[1]<0
				//tZero[2] = t
				//minNumber = 0
				//if _searchPos[1]<0
					//_phaseShift = 1.575-tZero[0]
				//else
					//_phaseShift = 3.15-tZero[0]
				//flag = true
			//else
				//do nothing (min not found)
			break;
		//_searchPos[0]=_searchPos[1]
	}

	return(flag);
}

float FPR_control::follow_command(char mode)
{
	float _gyroPosition;

	if(mode == 'p')
	{
		_gyroPosition = _gyroPos[0];
	}
	else if(mode == 'y')
	{
		_gyroPosition = _gyroPos[1];
	}
	else //mode == "r"
	{
		_gyroPosition = _gyroPos[2];
	}

	_tablePos = SCALE*sin(OMEGA*(_time + _phaseShift));
	
	return( _tablePos*(1.0 - _tableAlpha) + _gyroPosition*_tableAlpha );
}


void FPR_control::add_time(unsigned long milliseconds)
{
	_time = _time + (milliseconds/1000.0);
}


void FPR_control::update_gyroPos(float xPos, float yPos, float zPos)
{
	_gyroPos[0] = xPos;
	_gyroPos[1] = yPos;
	_gyroPos[2] = zPos;
}

*/