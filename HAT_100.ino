// Arduino sketch for MPU6050 using DMP MotionApps v4.1
// HAT V2.00 26/06/2014 by samtheeagle
// HAT V1.00 14/04/2013 by FuraX49
//
// Head Arduino Tracker (HAT) for FaceTrackNoIR 
//   http://facetracknoir.sourceforge.net/home/default.htm	
// I2C device class (I2Cdev)
//   https://github.com/jrowberg/i2cdevlib

#include <avr/eeprom.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define EEPROM_BASE       0xA0
#define EEPROM_SIGNATURE  0x55
char version[] = "HAT V 2.00";
const float Rad2Deg = (180/M_PI);
const int quickCalibrationSteps = 200;
const float accThreshold = 40;
const int accDuration = 100;

unsigned long xPosDetectTime = 0;
unsigned long xNegDetectTime = 0;
unsigned long yPosDetectTime = 0;
unsigned long yNegDetectTime = 0;
unsigned long zPosDetectTime = 0;
unsigned long zNegDetectTime = 0;

typedef struct  {
  int16_t  begin  ;   // 2  Begin
  uint16_t cpt ;      // 2  Compteur Frame or Code info or error
  float    gyro[3];   // 12 [Y, P, R] Gyro - MPU6050 defines these as Yaw about Z axis, Pitch about Y axis, Roll about X axis
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
  double gyro_offset[3];  // [Y, P, R] Gyro
  double acc_offset[3];   // [x, y, z] Acc
} _sensor_data;

MPU6050 mpu;
volatile bool mpuInterrupt = false;  // Indicates whether MPU interrupt pin has gone high

bool dmpReady = false;               // Set true if DMP init was successful
bool dmpLoaded = false;              // Set true if DMP loaded  successfully
bool askQuickCalibrate = false;      // Set true if quick calibration is requested
int quickCalibrateStep =  0;         // The current quick calibrate stemp number.
uint8_t mpuIntStatus;                // Holds actual interrupt status byte from MPU
uint8_t devStatus;                   // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                 // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                  // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];              // FIFO storage buffer
Quaternion q;                        // [w, x, y, z] Quaternion container
VectorInt16 acc;                     // [x, y, z]    Accel sensor measurements
VectorFloat gravity;                 // [x, y, z]    Gravity vector
_hat_data_packet hatData;            // HAT data packet to send over serial connection
_msg_info msgInfo;                   // Message packet to send over serial connection
_sensor_data quickCalibrationData;   // Sensor data values calculated by quick calibration

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               PRINT SERIAL MESSAGE                       ===
// ================================================================
void PrintCodeSerial(uint16_t code, char msg[24], bool EOL ) {
  msgInfo.code=code;
  memset(msgInfo.msg,0x00,24);
  strcpy(msgInfo.msg,msg);
  if (EOL) msgInfo.msg[23]=0x0A;
  Serial.write((byte*)&msgInfo,30);
}

//void PrintCodeSerial(uint16_t code, const char msg[24], bool EOL ) {
//  PrintCodeSerial(code, msg, EOL);
//}

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
  
  // To test... 
  // Edit MPU6050_6Axis_MotionApps20.h - MPU6050::dmpInitialize()
  //   setIntEnabled(0x82); to enable the motion detection interrupt.
  //
  // Adjust setMotionDetectionThreshold() and setMotionDetectionDuration() to see what happens.

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    dmpLoaded=true;
    // Read custom offset values, saved into eeprom by calibration sketch, for the specific mpu-6050 device
    PrintCodeSerial(3005,"Reading EEPROM data...",true);
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
// ===           RESET QUICK CALIBRATION OFFSETS                ===
// ================================================================
void ResetQuickCalibrationOffsets() {
  quickCalibrationData.gyro_offset[0] = 0;
  quickCalibrationData.gyro_offset[1] = 0;
  quickCalibrationData.gyro_offset[2] = 0;
  quickCalibrationData.acc_offset[0] = 0;
  quickCalibrationData.acc_offset[1] = 0;
  quickCalibrationData.acc_offset[2] = 0;
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
    Serial.println("MPU accelerometers offsets loaded from EEPROM");
    Serial.print(eprom_data.acc_offset[0]);
    Serial.print(", ");
    Serial.print(eprom_data.acc_offset[1]);
    Serial.print(", ");
    Serial.println(eprom_data.acc_offset[2]);
    Serial.println("MPU gyroscopes offsets loaded from EEPROM");
    Serial.print(eprom_data.gyro_offset[0]);
    Serial.print(", ");
    Serial.print(eprom_data.gyro_offset[1]);
    Serial.print(", ");
    Serial.println(eprom_data.gyro_offset[2]);
  }
}

// ================================================================
// ===               HANDLE X ACCELERATION                      ===
// ================================================================
void HandleXAcc(unsigned long now, float accX) {
  if(abs(accX) < accThreshold) {  // If movement is below threshold.
    if(xPosDetectTime > 0) {      // If movement in the X positive direction was occurring.
      unsigned long delta = now - xPosDetectTime;
      if(delta > accDuration) {
        hatData.acc[0]++;
      }
    }
    if(xNegDetectTime > 0) {      // If movement in the X negative direction was occurring.
      unsigned long delta = now - xNegDetectTime;
      if(delta > accDuration) {
        hatData.acc[0]--;
      }
    }
    xPosDetectTime = 0;
    xNegDetectTime = 0;
  }
  else if(accX >= accThreshold && xPosDetectTime == 0) {  // Record X positive movement start time.
    xPosDetectTime = now;
  }
  else if(accX <= -accThreshold && xNegDetectTime == 0) { // Record X negative movement start time.
    xNegDetectTime = now;
  }
}

// ================================================================
// ===               HANDLE Y ACCELERATION                      ===
// ================================================================
void HandleYAcc(unsigned long now, float accY) {
  if(abs(accY) < accThreshold) {  // If movement is below threshold.
    if(yPosDetectTime > 0) {      // If movement in the Y positive direction was occurring.
      unsigned long delta = now - yPosDetectTime;
      if(delta > accDuration) {
        hatData.acc[1]++;
      }
    }
    if(yNegDetectTime > 0) {      // If movement in the Y negative direction was occurring.
      unsigned long delta = now - yNegDetectTime;
      if(delta > accDuration) {
        hatData.acc[1]--;
      }
    }
    yPosDetectTime = 0;
    yNegDetectTime = 0;
  }
  else if(accY >= accThreshold && yPosDetectTime == 0) {  // Record Y positive movement start time.
    yPosDetectTime = now;
  }
  else if(accY <= -accThreshold && yNegDetectTime == 0) { // Record Y negative movement start time.
    yNegDetectTime = now;
  }
}

// ================================================================
// ===               HANDLE Z ACCELERATION                      ===
// ================================================================
void HandleZAcc(unsigned long now, float accZ) {
  if(abs(accZ) < accThreshold) {  // If movement is below threshold.
    if(zPosDetectTime > 0) {      // If movement in the Z positive direction was occurring.
      unsigned long delta = now - zPosDetectTime;
      if(delta > accDuration) {
        hatData.acc[2]++;
      }
    }
    if(zNegDetectTime > 0) {      // If movement in the Z negative direction was occurring.
      unsigned long delta = now - zNegDetectTime;
      if(delta > accDuration) {
        hatData.acc[2]--;
      }
    }
    zPosDetectTime = 0;
    zNegDetectTime = 0;
  }
  else if(accZ >= accThreshold && zPosDetectTime == 0) {  // Record Z positive movement start time.
    zPosDetectTime = now;
  }
  else if(accZ <= -accThreshold && zNegDetectTime == 0) { // Record Z negative movement start time.
    zNegDetectTime = now;
  }
}

// ================================================================
// ===                    Serial Command                        ===
// ================================================================
void SerialEvent(){
  char command = (char)Serial.read();
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
    quickCalibrateStep=0;
    ResetQuickCalibrationOffsets();  
    askQuickCalibrate=true;
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
    	Serial.print(quickCalibrationData.gyro_offset[i]);
  	  Serial.println();
    }
    Serial.println("Accelerometers offsets");
    for (int i=0; i <= 2; i++) {
  	  Serial.print(i);
  	  Serial.print(" : ");
    	Serial.print(quickCalibrationData.acc_offset[i]);
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
  if(Serial.available() > 0)  SerialEvent();

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
      
//      Serial.print("\n");
//      Serial.print(hatData.gyro[0]);
//      Serial.print(", ");
//      Serial.print(hatData.gyro[1]);
//      Serial.print(", ");
//      Serial.print(hatData.gyro[2]);
//      Serial.print("\n");

      // Get real acceleration, adjusted to remove gravity
      VectorInt16 linearAcc;
      mpu.dmpGetAccel(&acc, fifoBuffer);
      mpu.dmpGetLinearAccel(&linearAcc, &acc, &gravity);
      
      hatData.acc[0]= acc.x / 1000.0;
      hatData.acc[1] = acc.y / 1000.0;
      hatData.acc[2] = acc.z / 1000.0;
//      unsigned long now = millis();
//      HandleXAcc(now, linearAcc.x / 10.0);
//      HandleYAcc(now, linearAcc.y / 10.0);
//      HandleZAcc(now, linearAcc.z / 10.0);
      
      if (askQuickCalibrate) {
        if (quickCalibrateStep >= quickCalibrationSteps) {
          quickCalibrateStep=0;
          quickCalibrationData.gyro_offset[0] = quickCalibrationData.gyro_offset[0] / quickCalibrationSteps;
          quickCalibrationData.gyro_offset[1] = quickCalibrationData.gyro_offset[1] / quickCalibrationSteps;
          quickCalibrationData.gyro_offset[2] = quickCalibrationData.gyro_offset[2] / quickCalibrationSteps;
          askQuickCalibrate=false;
        } 
        else {
          quickCalibrationData.gyro_offset[0] += (float) hatData.gyro[0];
          quickCalibrationData.gyro_offset[1] += (float) hatData.gyro[1];
          quickCalibrationData.gyro_offset[2] += (float) hatData.gyro[2];
          quickCalibrateStep++;
        }
      }

      // Conversion angles Euler en +-180 Degrees
      for (int i=0; i <= 2; i++) {
        hatData.gyro[i]= (hatData.gyro[i] - quickCalibrationData.gyro_offset[i] ) * Rad2Deg;
        if  (hatData.gyro[i]>180) {
          hatData.gyro[i] = hatData.gyro[i] - 360;
        }
      }

      if (askQuickCalibrate) {
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
