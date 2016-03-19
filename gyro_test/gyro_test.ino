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

L3G gyro;

unsigned long lastLoopTime = millis();
unsigned long lastLoopTimePrint = millis();
unsigned long loopTime, loopTimePrint;


float xrate = 0;
float xpos = 0;

  int x_noise[500];
  int y_noise[500];
  int z_noise[500];
  int x_bias = 0;
  int y_bias = 0;
  int z_bias = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  
  
  
  for(int i = 0; i<500; i++)
  {
	gyro.read();
	x_noise[i] = (int)gyro.g.x;
	y_noise[i] = (int)gyro.g.y;
	z_noise[i] = (int)gyro.g.z;
  }
  for(int i = 0; i<500; i++)
  {
	x_bias = x_bias + x_noise[i];
	y_bias = y_bias + y_noise[i];
	z_bias = z_bias + z_noise[i];
  }
  
  x_bias = x_bias/500.0;
    
}

void loop() 
{
	loopTime = millis() - lastLoopTime;
	
	if( loopTime >= 2)
	{
		lastLoopTime = millis();
		
		gyro.read();
		
		xrate = lpf((((int)gyro.g.x) - x_bias), xrate, .05)*.00875*22.5;	
		//xrate = lpf((((int)gyro.g.x) - x_bias), xrate, .05)*.1575;	
		xpos = integral(xrate);
		
	}

	loopTimePrint = millis() - lastLoopTimePrint;
	
	if(loopTimePrint >= 500)
	{
		loopTimePrint = millis();
		Serial.print(millis()); Serial.print("\t"); Serial.print(xrate); Serial.print("\t"); Serial.println(xpos);
	}
	
}

float integral(float signal)
{
	static float integral = 0;
	static float signal_d1 = 0;
	
	integral = integral + (.002/2.0)*(signal + signal_d1);
	
	signal_d1 = signal;
	
	return(integral);
}

float lpf(float signal, float old, float alpha)
{
	return(old*(1.0-alpha) + signal*(alpha));
}