/*
 Omni-3MD 3 motor controller from botnroll.com http://www.botnroll.com/en/controllers/84-bot-n-roll-omni-3md-.html
 
 List of available routines to interact with the Omni-3MD:
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.7.10 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.74 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.

Purpose:
 This example lists all Omni3MD routines and has commented information about them.
 This examples lists all parameters that can be read from Omni3MD with commented information about them.
 This example is not intended to be uploaded to your arduino.
 
 Routines defined at Omni3MD.h:
  //setup routines
  void i2cConnect(byte omniAddress);
  void setI2cAddress (byte newAddress);
  void setI2cTimeout (byte timeout); //timeout x 100 miliseconds to receive i2C Commands
  void calibrate(boolean way1,boolean way2,boolean way3);
  void setPID(int Kp, int Ki, int Kd);
  void setPositionalPID(int Kp, int Ki, int Kd);
  void setRamp(int slope, int Kl);
  void setEncValue(byte encoder, unsigned int encValue);
  void setEncPrescaler(byte encoder, byte value);
  void setDifferential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);
  void setMinBat(float minbat);
  //reading routines
  float readTemperature();
  float readBattery();
  float readFirmware();
  void readFirmware(byte*,byte*,byte*);
  byte readControlRate();
  int readEnc1();
  int readEnc2();
  int readEnc3();
  int readEnc1Max();
  int readEnc2Max();
  int readEnc3Max();
  int readLim1();
  int readLim2();
  int readLim3();
  void readEncoders(int*,int*,int*);
  void readMovData(int*,int*,int*,float*,float*);
  void readAllData(int*,int*,int*,float*,float*,byte*,byte*,byte*,byte*,int*,int*,int*);
  //movement routines
  void movOmni(byte linear_speed,int rotational_speed,int direction);
  void movDifSi(double linear_speed,double rotational_speed);
  void movPosition(byte motor,unsigned int speed,unsigned int encPosition);
  void mov3mPid(int speed1,int speed2,int speed3);
  void mov1mPid(byte motor,int speed);
  void mov3m(int speed1,int speed2,int speed3);
  void mov1m(byte motor,int speed);
  void stop();
  void savePosition();
  void findHome(byte motor);
 
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
 on 26 September 2012
 Updated on 17 January 2017
 
 This code example is in the public domain. 
 http://www.botnroll.com
 */
 
#include <Wire.h>                   //required by BnrOmni.cpp
#include <BnrOmni.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        //default factory address
#define BROADCAST_ADDRESS 0x00      //i2c broadcast address

BnrOmni omni;                       //declaration of object variable to control the Omni3MD

/*** variable declarations ***/
//Variables to read from Omni3MD
float battery=0;       // battery reading
float temperature=0;   // temperature reading
float firmware=0;      // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_int=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dec=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dev=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte ctrl_rate;        // the control rate for your motors defined at calibration (in times per second)
int enc1=0;            // encoder1 reading, this is the encoder incremental count for the defined prescaler (positional control)
int enc2=0;            // encoder2 reading, this is the encoder incremental count for the defined prescaler (positional control)
int enc3=0;            // encoder3 reading, this is the encoder incremental count for the defined prescaler (positional control)
int enc1_max;          // maximum count for encoder 1 at calibration, for the defined control rate
int enc2_max;          // maximum count for encoder 2 at calibration, for the defined control rate
int enc3_max;          // maximum count for encoder 3 at calibration, for the defined control rate
unsigned int limiar1=0;         // Necessary motor M1 power to start movement. Acquired during motors calibration.
unsigned int limiar2=0;         // Necessary motor M2 power to start movement. Acquired during motors calibration.
unsigned int limiar3=0;         // Necessary motor M3 power to start movement. Acquired during motors calibration.


void setup()
{
  //setup routines
      omni.i2cConnect(OMNI3MD_ADDRESS);    //set i2c connection

      omni.setI2cAddress(OMNI3MD_ADDRESS); // change Omni3MD I2C address [byte newAddress]
  
      omni.setI2cTimeout(0);               // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
  
      omni.calibrate(1,1,1);               // send the calibration command to configure the OMmni3MD board
      delay(20000);                        // wait about 20s for calibration to end

      omni.setPid(1000,400,500);           // adjust parameters for Speed PID control [word Kp, word Ki, word Kd]
  
      omni.setPositionalPid(1000,400,500); // adjust parameters for Position PID control [word Kp, word Ki, word Kd]
  
      omni.setRamp(35,900);                // set acceleration ramp and limiar movement parameter gain[int slope, int Kl] slope between 0 and 100 kl->gain for necessary motor power to start movement
  
       //setEncPrescaler(byte encoder, byte value)  value: 0 - 1 pulse; 1 - 10 pulses; 2 - 100 pulses; 3 - 1000 pulses; 4 - 10000 pulses  (requires 10ms)
      omni.setEncPrescaler(1, 2);          // set Motor1 prescaler to 100; encoder count will increment by 1 each 100 pulses [byte encoder, byte value]
  
      omni.setEncValue(1,100);               // set Motor1 encoder value to 0 [byte encoder, word encValue]
      
      omni.setDifferential(97.5,37.5,16,14);//set differential movement SI parameters (Robot axis radius(mm),Wheel radius(mm),motor gearbox reduction,encoder cpr(1 channel))
 
      omni.setMinBat(9.7);                 // battery discharge protection voltage (Lithium 3S)
   
    
  //reading routines
      temperature=omni.readTemperature();  // read temperature value
      battery=omni.readBattery();          // read battery value
      firmware=omni.readFirmware();        // read firmware version
      ctrl_rate=omni.readControlRate();    // read the control rate value
      enc1=omni.readEnc1();                // read encoder1 count value for the defined prescaler (positional control)
      enc2=omni.readEnc2();                // read encoder1 count value for the defined prescaler (positional control)
      enc3=omni.readEnc3();                // read encoder1 count value for the defined prescaler (positional control)
      enc1_max=omni.readEnc1Max();         // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      enc2_max=omni.readEnc2Max();         // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      enc3_max=omni.readEnc3Max();         // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      limiar1=omni.readLim1();             // read the necessary motor power to start movement, acquired during motors calibration.
      limiar2=omni.readLim2();             // read the necessary motor power to start movement, acquired during motors calibration.
      limiar3=omni.readLim3();             // read the necessary motor power to start movement, acquired during motors calibration.
  
      // read all encoders at once (in a single I2C request)
      omni.readEncoders(&enc1,&enc2,&enc3);
  
      // read all encoders, battery and temperature at once (in a single I2C request)
      omni.readMovData(&enc1,&enc2,&enc3,&battery,&temperature);

      //reading all data values at once
      omni.readAllData(&enc1,&enc2,&enc3,&battery,&temperature,&firm_int,&firm_dec,&firm_dev,&ctrl_rate,&enc1_max,&enc2_max,&enc3_max,&limiar1,&limiar2,&limiar3);

  //movement routines
      omni.movOmni(0,-30,0);      // [byte linear_speed,int rotational_speed,int direction]
      omni.movDifSi(0.52,0.0);    // [double linear_speed,double rotational_speed]
      omni.movPosition(1,30,500); // move motor 1, CW at speed 30% until encoder reaches 500 counts and then brake [byte motor,int speed,int encPosition,byte stoptorque]
      omni.mov3mPid(30,-50,0);    // [int speed1,int speed2,int speed3]
      omni.mov1mPid(1,60);        // [byte motor,int speed]
      omni.mov3m(-60,0,60);       // [int speed1,int speed2,int speed3]
      omni.mov1m(1,100);          // [byte motor,int speed]
      omni.stop();                // stop all motors at once
      omni.savePosition();        // save encoders positional data to eeprom
      omni.findHome(2);           // find and reset home position for motor2 on positional movement
}
void loop()
{
}
