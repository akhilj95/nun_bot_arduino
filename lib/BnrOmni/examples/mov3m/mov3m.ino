/*
 Omni-3MD 3 motor controller from botnroll.com http://www.botnroll.com/en/controllers/84-bot-n-roll-omni-3md-.html
 
 Move 3 motors using Omni3MD:
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.7.10 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.74 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.

 Purpose:
  This example demonstrates linear movement of 3 motors without PID control
 
 Send movement orders: 
   Function: mov3m( speed1, speed2 , speed3 )
   - speed1: the desired speed for motor1 (-100 to +100).
   - speed2: the desired speed for motor2 (-100 to +100).
   - speed3: the desired speed for motor3 (-100 to +100).
   Ex: omni.mov3m(35,-45,55);
   The 3 motors will run at the desired speed.
 
 Important routines in this example:
      //setup routines
      void i2cConnect(byte omniAddress);
      void setI2cTimeout (byte timeout); //timeout x 100 milliseconds to receive i2C Commands
            
      //movement routines
      void mov3m(int speed1,int speed2,int speed3);

 Adjust the acceleration ramp if necessary:
   Function: setRamp( slope , kl )
   - slope: defines the acceleration of the motors (ramp inclination) varies from 0 to 100. Lower values slower acceleration.
   - kl: factor for the the necessary power to overcome the motor gearbox inertia, obtained during calibration from a system with encoders!
     Power*kl is loaded for the integral error of the PID control, under certain conditions, by the control algorithm.
     Positive values between 0 and 1000 for kl. The value 0 disables this function.
 
 
 Omni3MD board features: 
 - Holonomic movement integrated control for 3 motors with PID control;
 - Differential movement (SI units) with PID control;
 - Linear movement of 3 independent motors with or without PID control;
 - Position control of 3 motors using motor encoders.
 - Encoder reading;
 - Battery reading;
 - Temperature reading;
 
 
 The circuit:
 * Omni3MD board attached to Arduino analog input 4 (SDA) and 5 (SCL), GND and 5V DC.
 
 This example was created by José Cruz (www.botnroll.com)
 on 27 September 2012
 Updated on 17 January 2017
 
 This code example is in the public domain. 
 http://www.botnroll.com
*/
 
#include <Wire.h>                   // required by BnrOmni.cpp
#include <BnrOmni.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        // default factory address
#define M1  1            //Motor1
#define M2  2            //Motor2
#define M3  3            //Motor3

BnrOmni omni;          //declaration of object variable to control the Omni3MD


void setup()
{
    //setup routines
    delay(1500);                      // wait electronics to stabilize
    Serial.begin(57600);              // set baud rate to 57600bps for printing values on serial monitor. Press (ctrl+shift+m) after uploading
    omni.i2cConnect(OMNI3MD_ADDRESS); // set i2c connection
    omni.stop();                      // stop all motors
    omni.setI2cTimeout(0);            // safety parameter -> I2C communication must occur every [byte timeout] x 100 milliseconds for motor movement                             
    omni.setRamp(30,900);             // set acceleration ramp and limiar movement parameter gain[int slope, int Kl] slope between 0 and 100 kl->gain for necessary motor power to start movement
    omni.setMinBat(12.0);             // Battery discharge protection voltage (Lithium 4S)
    
    Serial.print("Firmware:");  
    Serial.println(omni.readFirmware());      // print firmware value
    Serial.print("ControlRate:");
    Serial.println(omni.readControlRate());   // print control rate value
    Serial.print("Enc1max:");
    Serial.println(omni.readEnc1Max());       // print encoder1 maximum calibration value
    Serial.print("Enc2max:");
    Serial.println(omni.readEnc2Max());       // print encoder2 maximum calibration value 
    Serial.print("Enc3max:");
    Serial.println(omni.readEnc3Max());       // print encoder3 maximum calibration value
    Serial.print("Battery:");
    Serial.println(omni.readBattery());       // print battery value
    Serial.print("Temperature:");
    Serial.println(omni.readTemperature());   // print temperature value 
}


void loop()
{
    //Variables for motion control
    int speed1=65;
    int speed2=70;
    int speed3=80;
    
    //Move motors    
    omni.mov3m(speed1,speed2,speed3);    // move motors at desired speed
    delay(4000);
    
    omni.stop();                         // stop all motors
    delay(2000);

    omni.mov3m(-speed1,-speed2,-speed3); // move motors at desired speed
    delay(4000);
 
    omni.mov3m(0,0,0);                   // stop motors (using acceleration ramp)
    delay(2000);
}
