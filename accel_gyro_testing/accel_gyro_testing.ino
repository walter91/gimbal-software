/*
The sensor outputs provided by the library are the raw
16-bit values obtained by concatenating the 8-bit high and
low accelerometer and gyro data registers. They can be
converted to units of g and dps (degrees per second) using
the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).
Example: An LSM6DS33 gives an accelerometer Z axis reading
of 16276 with its default full scale setting of +/- 2 g. The
LA_So specification in the LSM6DS33 datasheet (page 11)
states a conversion factor of 0.061 mg/LSB (least
significant bit) at this FS setting, so the raw reading of
16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/

#include <Wire.h>
#include <LSM6.h>
#include <math.h>

#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131

LSM6 imu;

char report[80];

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop()
{
    int Ts = 10;
    float alpha = .05;
    unsigned long loopTime, lastLoop;
    float eulerAng[] = {0.0, 0.0, 0.0};
    float phi, theta, psi;  //roll, pitch, yaw
    float acc[3];
    float gyro[3];

    lastLoop = millis();

    while(1)
        {
        loopTime = millis() - lastLoop;

        if(loopTime >= Ts)
        {
            lastLoop = millis();
            imu.read();

            // snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
            //   imu.a.x, imu.a.y, imu.a.z,
            //   imu.g.x, imu.g.y, imu.g.z);
            // Serial.println(report);

            gyro[0] = imu.g.x*0.00875;
            gyro[1] = imu.g.y*0.00875;
            gyro[2] = imu.g.z*0.00875;

            acc[0] = imu.a.x*0.000061;
            acc[1] = imu.a.y*0.000061;
            acc[2] = imu.a.z*0.000061;

            estimate_imu(eulerAng, acc, gyro, loopTime/1000.0, .05);

            phi = eulerAng[0];
            theta = eulerAng[1];
            psi = eulerAng[2];

            //Serial.println(millis());
            Serial.print(phi);
            Serial.print("\t");
            Serial.print(theta);
            Serial.print("\t");
            Serial.print(psi);
            Serial.println();
        }
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

    float phi_a, theta_a;

   float forceMagEst = sqrt(xdot*xdot + ydot*ydot + zdot*zdot);
    if(0.5 < forceMagEst && forceMagEst < 2.0)
    {
        phi_a = atan2(ydot, zdot)*RAD2DEG;
        phi = phi_a*(1.0 - alpha) + phi_g*(alpha);

        //theta_a = atan2(xdot, zdot)*RAD2DEG;
        theta_a = asin(xdot)*RAD2DEG;
        theta = theta_a*(1.0 - alpha) + theta_g*(alpha);

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

float lpf(float signal, float old, float alpha)
{
    return(old*(1.0-alpha) + signal*(alpha));
}

