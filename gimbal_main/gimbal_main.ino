

#include <Wire.h>
#include "L3G.h"
#include "Control.h"

#define DEG2RAD 0.01745329251

typedef enum STATE{direct, pitching, yawing, rolling};
typedef enum subSTATE{waiting, finding, following};



void setup()
{
	
}

void loop()
{
	
	while(1)
	{
		
		switch(STATE)
		{
			case(direct):
				
				break;
			case(pitching):
				switch(subSTATE)
				{
					case(waiting):
						//do nothing until the button is pushed again
						break;
					case(finding):
						if(elevation.phase_found("p"))	//the phase shift offset has been updated for pitch
						{
							subSTATE = following;
							flag = true;
						}
						break;
					case(following):	//Update the position values
						pitchCommand = elevation.follow_command("p");
						pitchPosition = elevation.state(eleCount, "p");
						pitchControl = elevation.pid(pitchCommand, pitchPosition, flag);
						elevation.move(pitchControl);
						if(pitchFlag)
						{
							flag = false;
						}
						break;
				}
				break;
			case(yawing):
				switch(subSTATE)
				{
					case(waiting):
					
						break;
					case(finding):
					
						break;
					case(following):	//Update the position values
					
						break;
				}
				break;
			case(rolling):
				switch(subSTATE)
				{
					case(waiting):
					
						break;
					case(finding):
					
						break;
					case(following):	//Update the position values
					
						break;
				}
				break;
		}
		
	}
	
}

void button1()
{
	STATE = direct;
	azimuth.stop();		//turn off PWM and set command to position (stay where you are)
	elevation.stop();	//turn off PWM and set command to position (stay where you are)
}

void button2()
{
	if(STATE == direct)
	{
		STATE = pitching;	//Update the state
		subSTATE = waiting;	//Wait for second button push before starting anything
		azimuth.zero();		//Make the current position zero for the azimuth encoder
		elevation.zero();	//Make the current position zero for the elevation encoder
		gyro.zero();		//Make the current position zero for the gyro
		gyro.calibrate();	//Recalibrate the gyro in all three degrees to remove the bias
	}
	else if(STATE == pitching)
	{
		subState = finding;
		elevation._time = 0.0;
	}
	else
	{
		//your not in the right state to change
	}
}

void button3()
{
	if(STATE == direct)
	{
		STATE = yawing;	//Update the state
		subSTATE = waiting;	//Wait for second button push before starting anything
		azimuth.zero();		//Make the current position zero for the azimuth encoder
		elevation.zero();	//Make the current position zero for the elevation encoder
		gyro.zero();		//Make the current position zero for the gyro
		gyro.calibrate();	//Recalibrate the gyro in all three degrees to remove the bias
	}
	else if(STATE == yawing)
	{
		subState = finding;
		azimuth._time = 0.0;
	}
	else
	{
		//your not in the right state to change
	}
}

void button4()
{
	if(STATE == direct)
	{
		STATE = rolling;	//Update the state
		subSTATE = waiting;	//Wait for second button push before starting anything
		azimuth.zero();		//Make the current position zero for the azimuth encoder
		elevation.zero();	//Make the current position zero for the elevation encoder
		gyro.zero();		//Make the current position zero for the gyro
		gyro.calibrate();	//Recalibrate the gyro in all three degrees to remove the bias
	}
	else if(STATE == rolling)
	{
		subState = finding;
		elevation._time = 0.0;
	}
	else
	{
		//your not in the right state to change
	}
}