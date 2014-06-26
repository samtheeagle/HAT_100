// Arduino sketch for MPU6050 using DMP MotionApps v4.1 
// HAT 26/06/2014 by samtheeagle
// HAT 14/04/2013 by FuraX49
//
// Head Arduino Tracker for FaceTrackNoIR 
//   http://facetracknoir.sourceforge.net/home/default.htm	
// I2C device class (I2Cdev)
//   https://github.com/jrowberg/i2cdevlib

#include <avr/eeprom.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"

MPU6050 mpu;

typedef struct  {
  int16_t  begin  ;   // 2  Begin
  uint16_t cpt ;      // 2  Compteur Frame or Code info or error
  float    gyro[3];   // 12 [Y, P, R] Gyro
  float    acc[3];    // 12 [x, y, z] Acc
  int16_t  end ;      // 2  End
} _hat_data_packet;

typedef struct  {
  int16_t  begin  ;   // 2  Begin
  uint16_t code ;     // 2  Code info
  char     msg[24];   // 24 Message
  int16_t  end ;      // 2  End
} _msg_info;

typedef struct 
{
  byte   sig_hat;
  double gyro_offset[3];
  double acc_offset[3];
} _sensor_data;

#define EEPROM_BASE       0xA0
#define EEPROM_SIGNATURE  0x55

float Rad2Deg = (180/M_PI) ;

// MPU control/status vars
bool dmpReady = false;   // Set true if DMP init was successful
bool dmpLoaded = false;  // Set true if DMP loaded  successfuly
uint8_t mpuIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

char command;
char version[] = "HAT V 1.00";

// Orientation/motion vars
Quaternion q;           // [w, x, y, z] Quaternion container
VectorInt16 acc;        // [x, y, z]    Accel sensor measurements
VectorFloat gravity;    // [x, y, z]    Gravity vector

_hat_data_packet hatData;
_msg_info msgInfo;
_sensor_data calibrationData;

bool       askCalibrate = false;  // Set true when calibrating is requested
int        cptCal =  0;
const int  numCalibrationSteps = 200;

volatile bool mpuInterrupt = false;  // Indicates whether MPU interrupt pin has gone high

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               PRINT SERIAL FORMATTE                      ===
// ================================================================
void PrintCodeSerial(uint16_t code,char msg[24],bool EOL ) {
  msgInfo.code=code;
  memset(msgInfo.msg,0x00,24);
  strcpy(msgInfo.msg,msg);
  if (EOL) msgInfo.msg[23]=0x0A;
  Serial.write((byte*)&msgInfo,30);
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  // Join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // Initialize serial communication
  while (!Serial); // Wait for Leonardo enumeration, others continue immediately

  Serial.begin(115200);
  PrintCodeSerial(2000,version,true);

  hatData.begin=0xAAAA;
  hatData.cpt=0;
  hatData.end=0x5555;

  msgInfo.begin=0xAAAA;
  msgInfo.code=0;
  msgInfo.end=0x5555;

  // Initialize device
  PrintCodeSerial(3001,"Initializing I2C",true);
  mpu.initialize();

  // Verify connection
  PrintCodeSerial(3002,"Testing connections",true);

  if (mpu.testConnection()){
     PrintCodeSerial(3003,"MPU6050 connection OK",true);
  } else {
     PrintCodeSerial(9007,"MPU6050 ERRROR CNX",true);
  }

  while (Serial.available() && Serial.read()); // Empty buffer

  // Load and configure the DMP
  PrintCodeSerial(3004,"Initializing DMP...",true);
  devStatus = mpu.dmpInitialize();

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    dmpLoaded=true;
    // Read custom offset values, saved into eeprom by calibration sketch, for the specific mpu-6050 device
    PrintCodeSerial(3005,"Reading saved offset params from EEPROM...",true);
    ReadEepromOffsetData();
    
    // Turn on the DMP, now that it's ready
    PrintCodeSerial(3006,"Enabling DMP...",true);
    mpu.setDMPEnabled(true);
    
    // Enable Arduino interrupt detection
    PrintCodeSerial(3007,"Enabling interrupt",true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    PrintCodeSerial(5000,"HAT BEGIN",true);
    dmpReady = true;
    
    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();    
    // Empty FIFO
    fifoCount = mpu.getFIFOCount();
    while (fifoCount > packetSize) {
      fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, fifoCount);
    }
  } 
  else {
    // ERROR!
    // 1 = Initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    dmpLoaded=false;
    PrintCodeSerial(9000+devStatus,"DMP Initialization failed",true);
  }
}

// ================================================================
// ===               RESET CALIBRATION OFFSETS                  ===
// ================================================================
void ResetCalibrationOffsets() {
  calibrationData.gyro_offset[0] = 0;
  calibrationData.gyro_offset[1] = 0;
  calibrationData.gyro_offset[2] = 0;
  calibrationData.acc_offset[0] = 0;
  calibrationData.acc_offset[1] = 0;
  calibrationData.acc_offset[2] = 0;
}

// ================================================================
// ===               READ EEPROM OFFSET DATA                    ===
// ================================================================
void ReadEepromOffsetData() {
  _sensor_data eprom_data;
  eeprom_read_block( (void*)&eprom_data, (void*) EEPROM_BASE, sizeof(eprom_data));
  if (eprom_data.sig_hat == EEPROM_SIGNATURE) {
    mpu.setXAccelOffset(eprom_data.acc_offset[0]);
    mpu.setYAccelOffset(eprom_data.acc_offset[1]);
    mpu.setZAccelOffset(eprom_data.acc_offset[2]);
    mpu.setXGyroOffset(eprom_data.gyro_offset[0]);
    mpu.setYGyroOffset(eprom_data.gyro_offset[1]);
    mpu.setZGyroOffset(eprom_data.gyro_offset[2]);
    Serial.println("Accelerometers offsets loaded from EEPROM");
    Serial.print(eprom_data.acc_offset[0]);
    Serial.print(", ");
    Serial.print(eprom_data.acc_offset[1]);
    Serial.print(", ");
    Serial.println(eprom_data.acc_offset[2]);
    Serial.println("Gyroscopes offsets loaded from EEPROM");
    Serial.print(eprom_data.gyro_offset[0]);
    Serial.print(", ");
    Serial.print(eprom_data.gyro_offset[1]);
    Serial.print(", ");
    Serial.println(eprom_data.gyro_offset[2]);
  }
}

// ================================================================
// ===                    Serial Command                        ===
// ================================================================
void serialEvent(){
  command = (char)Serial.read();
  switch (command) {
  case 'S':
    PrintCodeSerial(5001,"HAT START",true);
    if (dmpLoaded==true) {
      mpu.resetFIFO();   
      hatData.cpt=0;              
      attachInterrupt(0, dmpDataReady, RISING);
      mpu.setDMPEnabled(true);
      dmpReady = true;
    } 
    else {
      PrintCodeSerial(9011,"Error DMP not loaded",true);
    }
    break;      

  case 's':
    PrintCodeSerial(5002,"HAT STOP",true);
    if (dmpReady==true) {
      mpu.setDMPEnabled(false);
      detachInterrupt(0);
      dmpReady = false;
    }
    break;      

  case 'R':
    PrintCodeSerial(5003,"HAT RESET",true);
    if (dmpLoaded==true) {
      mpu.setDMPEnabled(false);
      detachInterrupt(0);
      mpu.resetFIFO();   
      hatData.cpt=0;              
      dmpReady = false;
      setup();
    } 
    else {
      PrintCodeSerial(9011,"Error DMP not loaded",true);
    }
    break;      

  case 'C':
    cptCal=0;
    ResetCalibrationOffsets();  
    askCalibrate=true;
    break;      

  case 'V':
    PrintCodeSerial(2000,version,true);
    break;      

  case 'I':
  	Serial.println();
  	Serial.print("Version : \t");
    Serial.println(version);
    Serial.println("Gyroscopes offsets");
    for (int i=0; i <= 2; i++) {
  	  Serial.print(i);
  	  Serial.print(" : ");
    	Serial.print(calibrationData.gyro_offset[i]);
  	  Serial.println();
    }
    Serial.println("Accelerometers offsets");
    for (int i=0; i <= 2; i++) {
  	  Serial.print(i);
  	  Serial.print(" : ");
    	Serial.print(calibrationData.acc_offset[i]);
  	  Serial.println();
    }
    break;      
    
  default:
    break;
  }	
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // Leonardo BUG (simul Serial Event)
  if(Serial.available() > 0)  serialEvent();

  // If programming failed, don't try to do anything
  if (dmpReady)  { 

    while (!mpuInterrupt && fifoCount < packetSize) ;

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // Reset so we can continue cleanly
      mpu.resetFIFO();
      PrintCodeSerial(9010,"Overflow FIFO DMP",true);
      hatData.cpt=0;
      // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02) {
      // Wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // Read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // Track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // Get Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(hatData.gyro, &q, &gravity);

      // Get real acceleration, adjusted to remove gravity
      // not used in this script
      // mpu.dmpGetAccel(&acc, fifoBuffer);
      // mpu.dmpGetLinearAccel(&hatData.acc, &acc, &gravity);

      if (askCalibrate) {
        if ( cptCal>=numCalibrationSteps) {
          cptCal=0;
          calibrationData.gyro_offset[0] = calibrationData.gyro_offset[0] / numCalibrationSteps ;
          calibrationData.gyro_offset[1] = calibrationData.gyro_offset[1] / numCalibrationSteps ;
          calibrationData.gyro_offset[2] = calibrationData.gyro_offset[2] / numCalibrationSteps ;
          askCalibrate=false;
        } 
        else {
          calibrationData.gyro_offset[0] += (float) hatData.gyro[0];
          calibrationData.gyro_offset[1] += (float) hatData.gyro[1];
          calibrationData.gyro_offset[2] += (float) hatData.gyro[2];

          cptCal++;
        }
      }

      // Conversion angles Euler en +-180 Degrees
      for (int i=0; i <= 2; i++) {
        hatData.gyro[i]= (hatData.gyro[i] - calibrationData.gyro_offset[i] ) * Rad2Deg;
        if  (hatData.gyro[i]>180) {
          hatData.gyro[i] = hatData.gyro[i] - 360;
        }
      }

      if (askCalibrate) {
        hatData.gyro[0] = 0; 
        hatData.gyro[1] = 0;
        hatData.gyro[2] = 0;
        hatData.acc[0]= 0;
        hatData.acc[1] = 0;
        hatData.acc[2] = 0;
      }

      Serial.write((byte*)&hatData,30);

      hatData.cpt++;
      if (hatData.cpt>999) {
        hatData.cpt=0;
      }
    }
  }
  delay(1);
}
