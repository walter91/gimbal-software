

#include <Wire.h>
#include <LSM6.h>
#include "FPR_control.h"
#include <math.h>

#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131
#define THREAD_PITCH 0.0492126
#define GEAR_RATIO 3.0
#define LENGTH_HALF_SQ 9.0
#define SPREAD_0 5.0

FPR_control elevation;
FPR_control azimuth;

LSM6 imu;

typedef enum controls{direct, pitching, yawing, rolling};
controls STATE = direct;

typedef enum modes{waiting, following};
modes subSTATE = waiting;

long count_el = 0;
long count_az = 0;

const int A_az = 24;	//Pins for encoders
const int B_az = 25;	//Pins for encoders
const int A_el = 32;	//Pins for encoders
const int B_el = 33;	//Pins for encoders

bool A_az_state;
bool B_az_state;
bool A_el_state;
bool B_el_state;

float adcBits = 12.0;
float pwmBits = 12.0;

float ang[] = {0.0, 0.0};

bool flag = true;


void setup()
{
	Serial.begin(115200);
  	Wire.begin();

  	if (!imu.init())
  	{
	    Serial.println("Failed to detect and initialize IMU!");
	    while (1);
  	}
	imu.enableDefault();

	pinMode(A_az, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	pinMode(B_az, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	A_az_state = digitalRead(A_az);	//Initialize encoder state
	B_az_state = digitalRead(B_az);	//Initialize encoder state
	attachInterrupt(digitalPinToInterrupt(A_az), A_az_changed, CHANGE);	//Setup interrupt for background proccessing
    attachInterrupt(digitalPinToInterrupt(B_az), B_az_changed, CHANGE);	//Setup interrupt for background proccessing
	
	pinMode(A_el, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	pinMode(B_el, INPUT_PULLUP);	//Setup pin for encoder channel. Pullup for non-totempole output
	A_el_state = digitalRead(A_el);	//Initialize encoder state
	B_el_state = digitalRead(B_el);	//Initialize encoder state
	attachInterrupt(digitalPinToInterrupt(A_el), A_el_changed, CHANGE);	//Setup interrupt for background proccessing
    attachInterrupt(digitalPinToInterrupt(B_el), B_el_changed, CHANGE);	//Setup interrupt for background proccessing



}

void loop()
{
	//Main...

	//Pins
	const int pwm_az = 2;
	const int dir_az = 3;

	const int pwm_el = 8;
	const int dir_el = 9;

	//Control Variables
	const float kp = 1.0/15.0;
	const float ki = .01;
	const float kd = 0.0;

	const float intThreshHigh = 1.25;
	const float intThreshLow = 0.025;

	unsigned long Ts = 2;
	unsigned long Ts_w = 500;
	unsigned long lastLoopTime, loopTime, lastWriteLoopTime, writeLoopTime;

	float eulerAng[] = {0.0, 0.0, 0.0};
	float gyro[3];
	float acc[3];

	float alpha = 0.05;

	float encoderCPR_e = 4741.44;
	float encoderCPR_a = 4741.44;

	float motorAng_e, motorAng_a;
	float command_e, command_a;
	float position_e, position_a;
	float control_e, control_a;

	float com, pos;

	float pitchControl, pitchCommand, pitchPosition;
	float rollControl, rollCommand, rollPosition;
	float yawControl, yawCommand, yawPosition;

	float spread, height;

	elevation.set_gains(kp, ki, kd);
	elevation.set_pins(pwm_el, dir_el);
	elevation.set_antiwindup(intThreshHigh, intThreshLow);
	elevation.set_time_filter(Ts/1000.0, alpha, alpha);

	azimuth.set_gains(kp, ki, kd);
	azimuth.set_pins(pwm_az, dir_az);
	azimuth.set_antiwindup(intThreshHigh, intThreshLow);
	azimuth.set_time_filter(Ts/1000.0, alpha, alpha);


	while(1)
	{
		loopTime = millis() - lastLoopTime;
		if(loopTime >= Ts)
		{

			lastLoopTime = millis();
            imu.read();

			gyro[0] = imu.g.x*0.00875;
            gyro[1] = imu.g.y*0.00875;
            gyro[2] = imu.g.z*0.00875;

            acc[0] = imu.a.x*0.000061;
            acc[1] = imu.a.y*0.000061;
            acc[2] = imu.a.z*0.000061;

			estimate_imu(eulerAng, acc, gyro, loopTime/1000.0, alpha);

			switch(STATE)
			{
				case(direct):

					if (Serial.available())	//Serial Data is in the buffer...
					{
						Serial.println("Command Recieved");
						serial_parser();
					}

					command_a = ang[0];
					command_e = ang[1];

					motorAng_a = (float(count_az)*360.0)/(encoderCPR_a);
					position_a = -1.0*motorAng_a/GEAR_RATIO;

					control_a = azimuth.pid(command_a, position_a, flag);
					azimuth.dir(control_a);
					azimuth.pwm(control_a);

					motorAng_e = (float(count_el)*360.0)/(encoderCPR_e);
					spread = SPREAD_0 + THREAD_PITCH/(motorAng_e/360.0);	//Positive motor motion increases the spread
					height = 4.0*sqrt(LENGTH_HALF_SQ - pow(spread/2.0, 0.5));
					position_e = 90.0 - (((11.0 - height)/11.0)*105.0);
					
					com = command_e;
					pos = position_e;

					pitchControl = elevation.pid(command_e, position_e, flag);
					elevation.dir(control_e);
					elevation.pwm(control_e);

					if(flag)
					{
						flag = false;
					}

					break;

				case(pitching):
					switch(subSTATE)
					{
						case(waiting):
							//do nothing until the button is pushed again
							break;
						case(following):	//Update the position values
							pitchCommand = -1.0*eulerAng[0];
							motorAng_e = (float(count_el)*360.0)/(encoderCPR_e);
							spread = SPREAD_0 + THREAD_PITCH/(motorAng_e/360.0);	//Positive motor motion increases the spread
							height = 4.0*sqrt(LENGTH_HALF_SQ - pow(spread/2.0, 0.5));
							pitchPosition = 90.0 - (((11.0 - height)/11.0)*105.0);
							
							com = pitchCommand;
							pos = pitchPosition;

							pitchControl = elevation.pid(pitchCommand, pitchPosition, flag);
							elevation.dir(pitchControl);
							elevation.pwm(pitchControl);
							if(flag)
							{
								flag = false;
							}
							break;
					}
					break;

				case(rolling):
					switch(subSTATE)
					{
						case(waiting):
						
							break;
						case(following):	//Update the position values
							rollCommand = -1.0*eulerAng[1];
							motorAng_e = (float(count_el)*360.0)/(encoderCPR_e);
							spread = SPREAD_0 + THREAD_PITCH/(motorAng_e/360.0);	//Positive motor motion increases the spread
							height = 4.0*sqrt(LENGTH_HALF_SQ - pow(spread/2.0, 0.5));
							rollPosition = 90.0 - (((11.0 - height)/11.0)*105.0);
							
							com = rollCommand;
							pos = rollPosition;

							rollControl = elevation.pid(rollCommand, rollPosition, flag);
							elevation.dir(rollControl);
							elevation.pwm(rollControl);
							if(flag)
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
						case(following):	//Update the position values
							yawCommand = -1*eulerAng[2];
							motorAng_a = (float(count_az)*360.0)/(encoderCPR_a);
							yawPosition = -1.0*motorAng_a/GEAR_RATIO;
							
							pos = yawPosition;
							com = yawCommand;

							yawControl = azimuth.pid(yawCommand, yawPosition, flag);
							azimuth.dir(yawControl);
							azimuth.pwm(yawControl);
							if(flag)
							{
								flag = false;
							}
							break;
					}
					break;

			}
		}

		writeLoopTime = millis() - lastWriteLoopTime;
		if(writeLoopTime >= Ts_w)
		{
			Serial.print("DOF Command: "); Serial.print(com);
			Serial.print("\t");
			Serial.print("DOF Pos: "); Serial.println(pos);
			Serial.print("\t");
			Serial.print("DOF Error: "); Serial.println(com - pos);
			Serial.println("");
			lastWriteLoopTime = millis();
		}
		
	}
	
}

void button1()
{
	if(STATE == direct)
	{
		zero_az();
	}

	flag = true;
	STATE = direct;
	azimuth.stop();		//turn off PWM and set command to position (stay where you are)
	elevation.stop();	//turn off PWM and set command to position (stay where you are)

}

void button2()
{
	flag = true;
	if(STATE == direct)
	{
		STATE = pitching;	//Update the state
		subSTATE = waiting;	//Wait for second button push before starting anything
		zero_az();		//Make the current position zero for the azimuth encoder
	}
	else if(STATE == pitching)
	{
		if(subSTATE == waiting)
		{
			subSTATE = following;
		}
		else
		{
			subSTATE = waiting;
		}
	}
	else
	{
		//your not in the right state to change
	}
}

void button3()
{
	flag = true;
	if(STATE == direct)
	{
		STATE = yawing;	//Update the state
		subSTATE = waiting;	//Wait for second button push before starting anything
		zero_az();		//Make the current position zero for the encoders
	}
	else if(STATE == yawing)
	{
		if(subSTATE == waiting)
		{
			subSTATE = following;
		}
		else
		{
			subSTATE = waiting;
		}
	}
	else
	{
		//your not in the right state to change
	}
}

void button4()
{
	flag = true;
	if(STATE == direct)
	{
		STATE = rolling;	//Update the state
		subSTATE = waiting;	//Wait for second button push before starting anything
		zero_az();		//Make the current position zero for the azimuth encoder
	}
	else if(STATE == rolling)
	{
		if(subSTATE == waiting)
		{
			subSTATE = following;
		}
		else
		{
			subSTATE = waiting;
		}
	}
	else
	{
		//your not in the right state to change
	}
}

void estimate_imu(float state[3], float accData[3], float gyroData[3], float Ts, float alpha)
{

    static float phidot_d1 = 0;
    static float thetadot_d1 = 0;
    static float psidot_d1 = 0;

    static float xdot_d1 = 0;
    static float ydot_d1 = 0;
    static float zdot_d1 = 0;

  //phi, theta, psi = roll, pitch, yaw angles
  //p, q, r = roll, pitch, yaw rates

    float phi = state[0];
    float theta = state[1];
    float psi = state[2];
  
    float p = gyroData[0];
    float q = gyroData[1];
    float r = gyroData[2];

    float xdot = accData[0];
    float ydot = accData[1];
    float zdot = accData[2];

    //convert from body frame to euler angles
    float phidot = p + q*sin(phi*DEG2RAD)*tan(theta*DEG2RAD) + r*cos(phi*DEG2RAD)*tan(theta*DEG2RAD);
    float thetadot = q*cos(phi*DEG2RAD) - r*sin(phi*DEG2RAD);
    float psidot = q*(sin(phi*DEG2RAD)/cos(theta*DEG2RAD)) + r*(cos(phi*DEG2RAD)/cos(theta*DEG2RAD));

    //Ignore euler angle conversions
    // float phidot = p;
    // float thetadot = q;
    // float psidot = r;

    phidot = lpf(phidot, phidot_d1, alpha);
    thetadot = lpf(thetadot, thetadot_d1, alpha);
    psidot = lpf(psidot, psidot_d1, alpha);

    xdot = lpf(xdot, xdot_d1, alpha);
    ydot = lpf(ydot, ydot_d1, alpha);
    zdot = lpf(zdot, zdot_d1, alpha);
    
    float phi_g = integral(phidot, phidot_d1, phi, Ts);
    float theta_g = integral(thetadot, thetadot_d1, theta, Ts);
    float psi_g = integral(psidot, psidot_d1, psi, Ts);

    float phi_az, theta_az;

   float forceMagEst = sqrt(xdot*xdot + ydot*ydot + zdot*zdot);
    if(0.5 < forceMagEst && forceMagEst < 2.0)
    {
        phi_az = atan2(ydot, zdot)*RAD2DEG;
        phi = phi_az*(1.0 - alpha) + phi_g*(alpha);

        //theta_az = atan2(xdot, zdot)*RAD2DEG;
        theta_az = asin(xdot)*RAD2DEG;
        theta = theta_az*(1.0 - alpha) + theta_g*(alpha);

        psi = psi_g;
    }
    else
    {
        phi = phi_g;
        theta = theta_g;
        psi = psi_g;
    }

    if( (psidot_d1 * psidot) < 0.0)    //Zero crossing detection (velocity)
    {
        if(psidot < 0.0)    //Now negative (velocity)
        {
            psi = 5.0;  //Only because I know what the table is doing
        }
        else
        {
            psi = -5.0; //Only because I know what the table is doing
        }
    }

    //Update delayed signals
    phidot_d1 = phidot;
    thetadot_d1 = thetadot;
    psidot_d1 = psidot;

    xdot_d1 = xdot;
    ydot_d1 = ydot;
    zdot_d1 = zdot;

    state[0] = phi;
    state[1] = theta;
    state[2] = psi;
}   

float integral(float signal, float signal_d1, float integral, float dT)
{
    integral = integral + ((dT)*(signal + signal_d1)/2.0);
    
    signal_d1 = signal;
    
    return(integral);
}

void serial_parser()
{
	int index = Serial.parseInt();
	float angle_c = Serial.parseFloat();
	
	ang[index-1] = angle_c;

	Serial.flush();
}

float lpf(float signal, float old, float alpha)
{


    return(old*(1.0-alpha) + signal*(alpha));
}

void A_az_changed()
{
    A_az_state = digitalRead(A_az);
	//Serial.print("\t");
	//Serial.println("enc1A");
	
    if(A_az_state)  //A changed to HIGH
    {
        if(B_az_state)  //B is HIGH
        {
            count_az--;
        }
        else	//B is LOW
        {
            count_az++;
        }
    }
    else	//A changed to LOW
    {
        if(B_az_state)	//B is HIGH
        {
            count_az++;
        }
        else //B is LOW
        {
            count_az--;
        }
    }
}

void B_az_changed()
{
    B_az_state = digitalRead(B_az);
	//Serial.print("\t");
	//Serial.println("enc1B");
    if(B_az_state)  //B changed to HIGH
    {
        if(A_az_state)  //A is HIGH
        {
            count_az++;
        }
        else  //A is LOW
        {
            count_az--;
        }
    }
    else  //B changed to LOW
    {
        if(A_az_state)  //A is HIGH
        {
            count_az--;
        }
        else  //A is LOW
        {
            count_az++;
        }
    }
}

void A_el_changed()
{
    A_el_state = digitalRead(A_el);
	
	if(A_el_state)  //A changed to HIGH
    {
        if(B_el_state)  //B is HIGH
        {
            count_el--;
        }
        else	//B is LOW
        {
            count_el++;
        }
    }
    else	//A changed to LOW
    {
        if(B_el_state)	//B is HIGH
        {
            count_el++;
        }
        else	//B is LOW
        {
            count_el--;
        }
    }
}

void B_el_changed()
{
    B_el_state = digitalRead(B_el);
	
    if(B_el_state)  //B changed to HIGH
    {
        if(A_el_state)  //A is HIGH
        {
            count_el++;
			//Serial.println("enc2B, B high, A high, count_el++");
        }
        else  //A is LOW
        {
            count_el--;
			
			//Serial.println("enc2B, B High, A Low, count_el--");
        }
    }
    else  //B changed to LOW
    {
        if(A_el_state)  //A is HIGH
        {
            count_el--;
			//Serial.println("enc2B, B Low, A High, count_el--");
        }
        else  //A is LOW
        {
            count_el++;
			//Serial.println("enc2B, B Low, A Low, count_el++");
        }
    }
}

void zero_az()
{
	count_az = 0;
}
