/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).
Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/

#include <Wire.h>
#include "L3G.h"
#include "Control.h"

Control motor1;
Control motor2;

L3G gyro;


/************************
Global Variables
*************************/
bool enc1AState, enc1BState, enc2AState, enc2BState;	//Encoder channel states
const int enc1A = 24;	//Pins for encoders
const int enc1B = 25;	//Pins for encoders
const int enc2A = 32;	//Pins for encoders
const int enc2B = 33;	//Pins for encoders

long count1, count2;	//Global variables to track position of motors (encoders)

float adcBits = 12.0;
float pwmBits = 12.0;
float encoder1CPR = 4741.44;
float encoder2CPR = 4741.44;

float ang[] = {0.0, 0.0};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  /* if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault(); */
  
  //Global Setup
	count1 = 0;	//Initialize encoder count
	count2 = 0;	//Initialize encoder count
	
	pinMode(enc1A, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	pinMode(enc1B, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	enc1AState = digitalRead(enc1A);	//Initialize encoder state
	enc1BState = digitalRead(enc1B);	//Initialize encoder state
	attachInterrupt(digitalPinToInterrupt(enc1A), enc1A_changed, CHANGE);	//Setup interrupt for background proccessing
    attachInterrupt(digitalPinToInterrupt(enc1B), enc1B_changed, CHANGE);	//Setup interrupt for background proccessing
	
	pinMode(enc2A, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	pinMode(enc2B, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	enc2AState = digitalRead(enc2A);	//Initialize encoder state
	enc2BState = digitalRead(enc2B);	//Initialize encoder state
	attachInterrupt(digitalPinToInterrupt(enc2A), enc2A_changed, CHANGE);	//Setup interrupt for background proccessing
    attachInterrupt(digitalPinToInterrupt(enc2B), enc2B_changed, CHANGE);	//Setup interrupt for background proccessing

	motor1.set_precision(pwmBits, adcBits);
	motor2.set_precision(pwmBits, adcBits);
	
	
  
}

void loop()
{

	const int motorPwmPin1 = 2;	//Define pins for motor PWM
	const int motorPwmPin2 = 8;	//Define pins for motor PWM
	
	const int motorDirPin1 = 3;	//Define pins for motor direction
	const int motorDirPin2 = 9;	//Define pins for motor direction
	
	unsigned long lastTime1 = millis();	//Initialize sampeling timing
	unsigned long lastTime2 = millis();	//Initialize sampeling timing
	unsigned long lastPrintTime = millis();
	unsigned long loopTime1, loopTime2;
	
	float deg1, deg2, deg1_c, deg2_c, deg1_d1, deg2_d1, control1, control2, vel1, vel2, vel1_d1, vel2_d1, acc1, acc2;	//Define all the float variables
	
	const int Ts = 2; //Sample period in milliseconds
	const float tau = .05;	//digital LPF coefficent
	
	const float intThresh1 = 3;	//Threshold for using integrator (deg)
	const float contThresh1 = 1.0;	//Threshold for control variable saturation (dec of bits)
	const float intThresh2 = 3;	//Threshold for using integrator (deg)
	const float contThresh2 = 1.0;	//Threshold for control variable saturation (dec of bits)
	
	bool flag1 = true;	//Is this the first time through?
	bool flag2 = true;	//Is this the first time through?
	
	//Conditions for Kp calculation
	const float maxErrorDeg1 = 15.0;
	const float maxErrorDeg2 = 15.0;
	
	//Constant variables for PID algorithm
	const float kp1 = 1.0/maxErrorDeg1;
	const float ki1 = 0.0;
	const float kd1 = 0.01;
	
	//Constant variables for PID algorithm
	const float kp2 = 1.0/maxErrorDeg2;
	const float ki2 = 0.0;
	const float kd2 = 0.01;
	
	motor1.set_gains(kp1, ki1, kd1, 'p');
	motor1.set_pins(motorPwmPin1, motorDirPin1);
	motor1.set_antiwindup(intThresh1, .025, 'p');
	motor1.set_time_filter(Ts/1000, .05, .05, 'p');
	
	motor2.set_gains(kp2, ki2, kd2, 'p');
	motor2.set_pins(motorPwmPin2, motorDirPin2);
	motor2.set_antiwindup(intThresh2, .025, 'p');
	motor2.set_time_filter(Ts/1000, .05, .05, 'p');
	
	while(1)
	{
		
		if (Serial.available())	//Serial Data is in the buffer...
		{
			Serial.println("Read New Command");
			serial_parser();
			Serial.println("New Command");
		}		
		
		loopTime1 = millis() - lastTime1;
		if( loopTime1 >= Ts)
		{
			lastTime1 = millis();
			deg1 = (float(count1)*360.0)/encoder1CPR;
			vel1 = (deg1 - deg1_d1)/((float(loopTime1)/1000.0));
			acc1 = (vel1 - vel1_d1)/((float(loopTime1)/1000.0));
			deg1_c = float(ang[1]);
			//deg1_c = deg1_c + 1;
			control1 = motor1.position(deg1, deg1_c, flag1);
			digitalWrite(motorDirPin1, direction1(control1));
			analogWrite(motorPwmPin1, (pow(2.0,pwmBits)-1)*abs(control1));
			if(flag1)
			{
				flag1 = !flag1;
			}
			deg1_d1 = deg1; 
			vel1_d1 = vel1;
		}
		
		loopTime2 = millis() - lastTime2;
		if( loopTime2 >= Ts)
		{
			lastTime2 = millis();
			deg2 = (count2*360.0)/encoder2CPR;
			vel2 = (deg2 - deg2_d1)/((float(loopTime2)/1000.0));
			acc2 = (vel2 - vel2_d1)/((float(loopTime2)/1000.0));
			deg2_c = ang[2];
			control2 = motor2.position(deg2, deg2_c, flag2);
			digitalWrite(motorDirPin2, direction2(control2));
			analogWrite(motorPwmPin2, (pow(2.0,pwmBits)-1)*abs(control2));
			if(flag2)
			{
				flag2 = !flag2;
			}
			deg2_d1 = deg2; 
			vel2_d1 = vel2;
		}
		if((millis() - lastPrintTime >= 1000))
		{
			Serial.print("Motor 1 Command: "); Serial.print(deg1_c); Serial.print("\t");
			Serial.print("Motor 2 Command: "); Serial.println(deg2_c);
			Serial.print("Motor 1 Position: "); Serial.print(deg1); Serial.print("\t");
			Serial.print("Motor 2 Position: "); Serial.println(deg2);
			Serial.print("Motor 1 Control: "); Serial.print(control1*100.0); Serial.print("\t");
			Serial.print("Motor 2 Control: "); Serial.println(control2*100.0);
			Serial.println("");
			lastPrintTime = millis();
			
			/* Serial.print("Motor 1 Pos: "); Serial.print(deg1); Serial.print("\t");
			Serial.print("Motor 2 Pos: "); Serial.println(deg2);
			Serial.print("Motor 1 Vel: "); Serial.print(vel1); Serial.print("\t");
			Serial.print("Motor 2 Vel: "); Serial.println(vel2);
			Serial.print("Motor 1 Acc: "); Serial.print(acc1); Serial.print("\t");
			Serial.print("Motor 2 Acc: "); Serial.println(acc2);
			Serial.println("");
			lastPrintTime = millis(); */
			
			/* Serial.print("Motor 1 Control: "); Serial.println(control1); //Serial.print("\t");
			//Serial.print("Motor 2 Pos: "); Serial.println(deg2);
			Serial.print("Motor 1 Vel: "); Serial.println(vel1); //Serial.print("\t");
			//Serial.print("Motor 2 Vel: "); Serial.println(vel2);
			Serial.print("Motor 1 Command: "); Serial.println(deg1_c); //Serial.print("\t");
			//Serial.print("Motor 2 Acc: "); Serial.println(acc2);
			Serial.println("");
			lastPrintTime = millis(); */
			
			/* Serial.print(deg1);Serial.print("\t");
			Serial.print(vel1);Serial.print("\t");
			Serial.println(acc1);
			lastPrintTime = millis();			*/
		}
	}	

}

void enc1A_changed()
{
    enc1AState = digitalRead(enc1A);
	//Serial.print("\t");
	//Serial.println("enc1A");
	
    if(enc1AState)  //A changed to HIGH
    {
        if(enc1BState)  //B is HIGH
        {
            count1--;
        }
        else	//B is LOW
        {
            count1++;
        }
    }
    else	//A changed to LOW
    {
        if(enc1BState)	//B is HIGH
        {
            count1++;
        }
        else //B is LOW
        {
            count1--;
        }
    }
}

void enc1B_changed()
{
    enc1BState = digitalRead(enc1B);
	//Serial.print("\t");
	//Serial.println("enc1B");
    if(enc1BState)  //B changed to HIGH
    {
        if(enc1AState)  //A is HIGH
        {
            count1++;
        }
        else  //A is LOW
        {
            count1--;
        }
    }
    else  //B changed to LOW
    {
        if(enc1AState)  //A is HIGH
        {
            count1--;
        }
        else  //A is LOW
        {
            count1++;
        }
    }
}

void enc2A_changed()
{
    enc2AState = digitalRead(enc2A);
    
	//Serial.println("enc2A");
	
	if(enc2AState)  //A changed to HIGH
    {
        if(enc2BState)  //B is HIGH
        {
            count2--;
        }
        else	//B is LOW
        {
            count2++;
        }
    }
    else	//A changed to LOW
    {
        if(enc2BState)	//B is HIGH
        {
            count2++;
        }
        else	//B is LOW
        {
            count2--;
        }
    }
}

void enc2B_changed()
{
    enc2BState = digitalRead(enc2B);
	
    if(enc2BState)  //B changed to HIGH
    {
        if(enc2AState)  //A is HIGH
        {
            count2++;
			//Serial.println("enc2B, B high, A high, count2++");
        }
        else  //A is LOW
        {
            count2--;
			
			//Serial.println("enc2B, B High, A Low, count2--");
        }
    }
    else  //B changed to LOW
    {
        if(enc2AState)  //A is HIGH
        {
            count2--;
			//Serial.println("enc2B, B Low, A High, count2--");
        }
        else  //A is LOW
        {
            count2++;
			//Serial.println("enc2B, B Low, A Low, count2++");
        }
    }
}

bool direction1(float control)
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
	return(dir);
}

bool direction2(float control)
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
	return(dir);
}

void serial_parser()
{
	int index = Serial.parseInt();
	float angle_c = Serial.parseFloat();
	
	ang[index] = angle_c;

	Serial.flush();
}