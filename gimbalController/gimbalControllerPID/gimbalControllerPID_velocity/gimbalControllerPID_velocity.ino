/************************

This code implements PID control on position of two dc
 motors. State knowledge is based on quadrature encoder
 feedback.
 
 Hardware required for implementation
	Arduino Due
	H-bridge motor driver with PWM and DIR pins (e.g. Pololu G2 High-Power Motor Driver 24v13)
	DC motor with encoder with CPR>>360 (99:1 Metal Gear motor 25Dx54L mm HP 12V with 48 CPR Encoder)
		Note: The CPR of the recommended motor is effective 4741 after the gearbox.
	Optional: Logic level shifter (most encoders use 5v, including the recommended. The DUE requires 3.3v logic)
	
Wiring
	1)There should be four wires to each encoder (A, B, 5v, GND)
		A and B connect to the selected pins (see enc1A - enc2B)
		GND is commoned with the Arduino GND pin
		5v can come from any source (including Arduino) as long as GND of the source is commoned with logic
	
	2)POTs 1 and 2 should each have three connections
		Connection 1 to 3.3v on Arduino
		Connection 2 to the pin indicated in code (see potPin1 and potPin2)
		Connection 3 to GND of the Arduino
		
	3)Follow the H-bridge wiring scheme from the manufacturer
		see https://www.pololu.com/product/2992 for the H-bridge selected
		and https://www.pololu.com/product/3219 for the motor selected
	
Tuning
	The kp values indicated in this code were calculated to saturate (give full throttle) to the 
	motor when there is an error of 5 degrees. This is appropriate for generally continuous command
	strings provided the rate of command change is within the motor operating frequency. 
	
	ki and kd values are initially zeroed.
	
	If the rise-time (the time required for an initial error to become acceptably small, ignoring overshoot)
	is acceptably small, then kp should remain its default value.
	
	If there is an unacceptable level of overshoot
	or settling time increase the kd value in increments of 5 until an the system settles quickly enough.
	
	Once kp and kd are chosen, add a disturbance force to the motor output. This can be a spring, or
	any external circumstance which causes an error in the output. Then slowly increase ki until the error is
	eliminated quickly enough. ki should be kept as small as possible as it can cause instabilities and degrade 
	the dynamic response of the system.
	

Other Notes
	The direction was arbitrarily chosen in the code. If the system appears unstable initially, change the
	initialization of "dir" within the direction function. This swaps which direction is positive and negative.

	
************************/


/************************
Global Variables
*************************/
bool enc1AState, enc1BState, enc2AState, enc2BState, usePot;	//Encoder channel states
const int enc1A = 24;	//Pins for encoders
const int enc1B = 25;	//Pins for encoders
const int enc2A = 28;	//Pins for encoders
const int enc2B = 29;	//Pins for encoders

const int potSwitchPin = 10;	//Pin for switch to choose between serial or POT

long count1, count2;	//Global variables to track position of motors (encoders)

float adcBits = 12.0;
float pwmBits = 12.0;
float encoder1CPR = 4741.44;
float encoder2CPR = 4741.44;

float ang[] = {0.0, 0.0};

/************************
	Setup...
*************************/
void setup()
{
	Serial.begin(115200);
	
	
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
	
	//ADC and PWM precision
	analogWriteResolution(pwmBits);
	analogReadResolution(adcBits);
	
	pinMode(potSwitchPin, INPUT_PULLUP);
	usePot = digitalRead(potSwitchPin);
	attachInterrupt(digitalPinToInterrupt(potSwitchPin), switch_changed, CHANGE);
}

/************************
	Loop...
*************************/
void loop()
{
	const int motorPwmPin1 = 2;	//Define pins for motor PWM
	const int motorPwmPin2 = 5;	//Define pins for motor PWM
	
	pinMode(motorPwmPin1, OUTPUT);	//Setup pins for motor PWM
	pinMode(motorPwmPin2, OUTPUT);	//Setup pins for motor PWM
	
	const int motorDirPin1 = 3;	//Define pins for motor direction
	const int motorDirPin2 = 6;	//Define pins for motor direction
	
	pinMode(motorDirPin1, OUTPUT);	//Setup pins for motor direction
	pinMode(motorDirPin2, OUTPUT);	//Setup pins for motor direction
	
	unsigned long lastTime1 = millis();	//Initialize sampeling timing
	unsigned long lastTime2 = millis();	//Initialize sampeling timing
	unsigned long lastPrintTime = millis();
	
	float deg1, deg2, deg1_c, deg2_c, control1, control2;	//Define all the float variables
	
	const int Ts = .01; //Sample period in seconds (equivilent to 10 milliseconds)
	const float tau = .00005;	//digital LPF coefficent
	
	const float intThresh1 = .95;	//Threshold for using integrator (deg)
	const float contThresh1 = 4090.0;	//Threshold for control variable saturation (dec of bits)
	const float intThresh2 = .5;	//Threshold for using integrator (deg)
	const float contThresh2 = 4095.0;	//Threshold for control variable saturation (dec of bits)
	
	bool flag1 = true;	//Is this the first time through?
	bool flag2 = true;	//Is this the first time through?
	
	//Conditions for Kp calculation
	const float maxErrorDeg1 = 5.0;
	const float maxErrorDeg2 = 5.0;
	
	//Constant variables for PID algorithm
	const float kp1 = 1.0/maxErrorDeg1;	//Should be .2 when using 5-degrees
	const float ki1 = 0.05;
	//const float kd1 = -0.00002;
	const float kd1 = 0.0;
	
	//Constant variables for PID algorithm
	const float kp2 = 1.0/maxErrorDeg2;	//Should be .2 when using 5-degrees
	const float ki2 = 0.0;
	const float kd2 = 0.0;

	const int potPin1 = A1;	//Define pins for Potentiometers
	const int potPin2 = A2;	//Define pins for Potentiometers
	
	pinMode(potPin1, INPUT);	//Setup pins for Potentiometers
	pinMode(potPin2, INPUT);	//Setup pins for Potentiometers
	
	while(1)
	{
		if (Serial.available())	//Serial Data is in the buffer...
		{
			Serial.println("Read New Command");
			serial_parser();
			Serial.println("New Command");
		}		
		
		if(millis() - lastTime1 >= (1.0/Ts))
		{
			lastTime1 = millis();
			deg1 = (float(count1)*360.0)/encoder1CPR;
			if(usePot)
			{
				deg1_c = 360.0*(analogRead(potPin1)/(pow(2.0,adcBits)-1));
			}
			else
			{
				deg1_c = float(ang[1]);
			}
			control1 = pid1(deg1, deg1_c, Ts, tau, kp1, ki1, kd1, intThresh1, contThresh1, flag1);
			digitalWrite(motorDirPin1, direction(control1));
			analogWrite(motorPwmPin1, (pow(2.0,pwmBits)-1)*abs(control1));
			if(flag1)
			{
				flag1 = !flag1;
			}
		}
		
		if(millis() - lastTime2 >= (1.0/Ts))
		{
			lastTime2 = millis();
			deg2 = (count2*360.0)/encoder2CPR;
			if(usePot)
			{
				deg2_c = 360.0*(analogRead(potPin2)/(pow(2.0,adcBits)-1));
			}
			else
			{
				deg2_c = float(ang[2]);
			}
			control2 = pid2(deg2, deg2_c, Ts, tau, kp2, ki2, kd2, intThresh2, contThresh2, flag2);
			digitalWrite(motorDirPin2, direction(control2));
			analogWrite(motorPwmPin2, (pow(2.0,pwmBits)-1)*abs(control2));
			if(flag2)
			{
				flag2 = !flag2;
			}
		}
		
		if(millis() - lastPrintTime >= 1000)
		{
			Serial.print("Motor 1 Command: "); Serial.print(deg1_c); Serial.print("\t");
			Serial.print("Motor 2 Command: "); Serial.println(deg2_c);
			Serial.print("Motor 1 Position: "); Serial.print(deg1); Serial.print("\t");
			Serial.print("Motor 2 Position: "); Serial.println(deg2);
			//Serial.print("Motor 1 Control: "); Serial.print(control1); Serial.print("\t");
			//Serial.print("Motor 2 Control: "); Serial.println(control2);
			Serial.println("");
			lastPrintTime = millis();
		}
	}	
}

/************************
Additional Functions
*************************/
float pid1(float state, float stateCommand, float Ts, float tau, float kp, float ki, float kd, float intThresh, float contThresh, bool flag)
{
	Ts = .01;
	
	static float integrator, differentiator, error_d1;
	
	if(flag)
	{
		integrator = 0.0;
		differentiator = 0.0;
		error_d1 = 0.0;
	}
	
	float error = stateCommand - state;
	
	if(abs(error) < intThresh && abs(error) > .25)
	{
		integrator = integrator + (Ts/2.0)*(error + error_d1);
	}
	else
	{
		integrator = 0.0;
	}
	
	//Serial.println(integrator);
	
	differentiator = ((2.0*tau - Ts)/(2.0*tau + Ts))*differentiator + (2.0/(2.0*tau + Ts))*(error - error_d1);
	
	//differentiator = (error_d1 - error)/Ts;
	
	/* Serial.print(state);
	Serial.print("\t");
	Serial.print(stateCommand);
	Serial.print("\t");
	Serial.print(error);
	Serial.print("\t");
	Serial.print(error_d1);
	Serial.print("\t");
	Serial.println(differentiator); */
	
	
	//differentiator = 0;
	
	error_d1 = error;
	
	return(saturate(kp*error + ki*integrator + kd*differentiator, contThresh));
	
}

float pid2(float state, float stateCommand, float Ts, float tau, float kp, float ki, float kd, float intThresh, float contThresh, bool flag)
{
	Ts = .01;
	
	static float integrator, differentiator, error_d1;
	
	if(flag)
	{
		integrator = 0.0;
		differentiator = 0.0;
		error_d1 = 0.0;
	}
	
	float error = stateCommand - state;
	
	if(error < intThresh)
	{
		integrator = integrator + (Ts/2.0)*(error + error_d1);
	}
	else
	{
		integrator = 0.0;
	}
	
	//Serial.println(integrator);
	
	differentiator = ((2.0*tau - Ts)/(2.0*tau + Ts))*differentiator + (2.0/(2.0*tau + Ts))*(error - error_d1);
	
	//differentiator = (error_d1 - error)/Ts;
	
	/* Serial.print(state);
	Serial.print("\t");
	Serial.print(stateCommand);
	Serial.print("\t");
	Serial.print(error);
	Serial.print("\t");
	Serial.print(error_d1);
	Serial.print("\t");
	Serial.println(differentiator); */
	
	
	//differentiator = 0;
	
	error_d1 = error;
	
	return(saturate(kp*error + ki*integrator + kd*differentiator, contThresh));
	
}


float saturate(float control, float controlThresh)
{
	if(control > controlThresh)
	{
		control = controlThresh;
	}
	else if(control < -controlThresh)
	{
		control = -controlThresh;
	}
	
	return(control);
}

void enc1A_changed()
{
    enc1AState = digitalRead(enc1A);
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
        }
        else  //A is LOW
        {
            count2--;
        }
    }
    else  //B changed to LOW
    {
        if(enc2AState)  //A is HIGH
        {
            count2--;
        }
        else  //A is LOW
        {
            count2++;
        }
    }
}

bool direction(float control)
{
	bool dir;
	
	if(control >= 0)
	{
		dir = 1;
	}
	else
	{
		dir = 0;
	}
	return(dir);
}

void serial_parser()
{
	int index = Serial.read() - 48;
	Serial.read();
	/* 
	int next = Serial.read();
	if(next == 45)
	{
		bool positive = false;
	}
	else
	{
		int firstDigit = next - 48;
	} */
	
	float angle_c = Serial.parseFloat();
	
	ang[index] = angle_c;

	Serial.flush();
}

void switch_changed()
{
	if(digitalRead(potSwitchPin))
	{
		usePot = true;
	}
	else
	{
		usePot = false;
	}
}


