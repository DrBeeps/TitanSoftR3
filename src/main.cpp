#include "structs.h"
#include "consts.h"

#include <BMI088.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#include <Orientation.h>
#include <pid.h>

Bmi088Accel accel(Wire, 0x19);
Bmi088Gyro gyro(Wire, 0x69);

Adafruit_BMP280 baro;
Adafruit_Sensor *baro_temp = baro.getTemperatureSensor();
Adafruit_Sensor *baro_pressure = baro.getPressureSensor();

// ====================
// PIN DEF
// ====================
const int chipSelect = BUILTIN_SDCARD;
const int spiFlashChipSelect = 1;

const int pyro1 = 2;
const int pyro2 = 10;
const int pyro3 = 29;
const int pyro4 = 33;

const int LEDR = 19; 
const int LEDG = 16;
const int LEDB = 15;


// ====================
// SERVO
// ====================
double SGR = 6;
Servo servoZ;
Servo servoY;
Servo escZ;


// ====================
// TIME
// ====================
uint64_t flightStartMicros;
uint64_t dataLogMicros;
uint64_t launchDetectStart;
uint64_t burnoutDetectStart;

uint64_t thisLoopMicros;

double dt;
double dtPID;
double lastPIDUpdate;

uint32_t nextServoWrite;
const uint8_t servoHz = 16;
const uint32_t servoWriteSpacing = 1000000 / servoHz;

uint32_t flightDurationTime;

// ====================
// ORI / PID
// ====================
Orientation ori;
EulerAngles gyroData;
EulerAngles gyroOut;

double LocalOrientationX, LocalOrientationY, LocalOrientationZ;
double accelX, accelY, accelZ;

int burnoutDetect = -11;
int launchDetect = 12;
uint64_t launchDetectTime = 2000;

float setpoint = 0;
float kp = 0.55;
float ki = 0.095;
float kd = 0.175;
PID y = {kp, ki, kd, setpoint};
PID z = {kp, ki, kd, setpoint};

double distToCOG;
double motorThrust;
double pidYOut, pidZOut;
double angleYOut, angleZOut;

FlightMode currentState;

// ====================
// LOGGING VARS
// ====================
File sdDataLog;

sensors_event_t tempEvent, pressureEvent;
// baro_temp->getEvent(&tempEvent);
// baro_pressure->getEvent(&pressureEvent);
double flightAlt;

/*
altitudeArray[altitudeIndex] = baro_pressure - terrainAltitude;
altitudeIndex = (altitudeIndex + 1) % 4;
altitude = (altitudeArray[0] + altitudeArray[1] + altitudeArray[2] + altitudeArray[3]) / 4;
*/

double fcBatt;

int DATA_ERROR;
int ABORT_CHECK;

// ====================
// SUB FUNCTIONS
// ====================

bool initAccel()
{
  if(accel.begin() < 0 || accel.setRange(Bmi088Accel::RANGE_12G) < 0 || accel.setOdr(Bmi088Accel::ODR_200HZ_BW_38HZ) < 0) return false;
}

bool initGyro()
{
  if(gyro.begin() < 0 || gyro.setRange(Bmi088Gyro::RANGE_1000DPS) < 0 || gyro.setOdr(Bmi088Gyro::ODR_400HZ_BW_47HZ) < 0) return false;
}

void writeServos(double sYV, double sZV)
{
  servoY.write(sYV + 90);
  servoZ.write(sZV + 90);
}

void serialLog()
{
  Serial.print("ROLL => "); Serial.print(gyroOut.roll); Serial.print("\t");
  Serial.print("PITCH => "); Serial.print(gyroOut.pitch);Serial.print("\t");
  Serial.print("YAW => "); Serial.print(gyroOut.yaw); Serial.print("\n");
}

// ====================
// MAIN FUNCTIONS
// ====================

void setupPins()
{
  servoY.attach(36);
  servoZ.attach(37);
  pinMode(pyro1, OUTPUT);
  pinMode(pyro2, OUTPUT);
  pinMode(pyro3, OUTPUT);
  pinMode(pyro4, OUTPUT);
}

void startup() 
{
  currentState = GROUND_IDLE;

  Serial.begin(115200);
  while(!Serial) delay(1);

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) 
  {
    return;
  }
  Serial.println("card initialized.");

  if (initAccel() < 0) 
  {
    Serial.println("Accel Initialization Error");
    while (1) {}
  }
  if (initGyro() < 0) 
  {
    Serial.println("Gyro Initialization Error");
    while (1) {}
  }
  while(!gyro.getDrdyStatus()) {}
}

void logFile()
{
  // TODO
}

void calibrate()
{
  // TODO!
}

void beginTiming()
{
  nextServoWrite = micros() + servoWriteSpacing;
  flightStartMicros = micros();
  thisLoopMicros = micros();
}

void updateTiming()
{
  dt = ((double)(thisLoopMicros - flightStartMicros) / 1000000.);
  flightStartMicros = thisLoopMicros;
}

void checkSensors()
{
  gyro.readSensor();
  accel.readSensor();
  
  gyroData.roll = gyro.getGyroX_rads();
  gyroData.pitch = gyro.getGyroY_rads();
  gyroData.yaw = gyro.getGyroZ_rads();

  accelX = accel.getAccelX_mss();
  accelY = accel.getAccelY_mss();
  accelZ = accel.getAccelZ_mss();

  ori.update(gyroData, dt);
  gyroOut = ori.toEuler();

  if (gyroOut.pitch >= 30 || gyroOut.pitch <= -30 || gyroOut.yaw >= 30 || gyroOut.yaw <= -30)
  {
    currentState = ABORT;
  }
}

void checkLaunch()
{
  if(accelX < launchDetect) launchDetectStart = thisLoopMicros;
  if(thisLoopMicros > launchDetectStart + launchDetectTime) currentState = POWERED_FLIGHT;
}

void updatePID()
{
  dtPID = (double)(thisLoopMicros - lastPIDUpdate) / 1000000.;
  lastPIDUpdate = thisLoopMicros;

  pidYOut = y.update(gyroData.pitch * RAD_TO_DEG, dtPID);
  pidZOut = z.update(gyroData.yaw * RAD_TO_DEG, dtPID);

  angleYOut = constrain(pidYOut, -5, 5);
  angleZOut = constrain(pidZOut, -5, 5);

  Serial.print("Y OUT => "); Serial.print(angleYOut); Serial.print("\t");
  Serial.print("Z OUT => "); Serial.print(angleZOut); Serial.print("\n");

  if (thisLoopMicros >= nextServoWrite)
  {
    writeServos(angleYOut * SGR, angleZOut * SGR);
    nextServoWrite += servoWriteSpacing;
  }
}

void checkBurnout()
{
  // if(accelX > burnoutDetect) burnoutDetectStart
  if ((flightStartMicros / 1000000.) >= 3.5)
  {
    currentState = UNPOWERED_FLIGHT;
  }
}

void checkApogee()
{
  // CHECK FOR APOGEE
}

void checkChuteStop()
{
  // CHECK FOR TIME TO DEPLOY CHUTES
}

void checkLanded()
{
  // CHECK ACCEL AND ALT FOR NON-MOVING MEASUREMENTS
}

void showSafe()
{
  // BLINK LIGHTS AND TURN OFF AND MOVE SPI FLASH DATA TO SD
}

void abort()
{
  writeServos(0, 0);
  digitalWrite(pyro3, HIGH); // This pyro controls the chutes
  ABORT_CHECK = 1;
  while(1) {}
}

// ====================
// SETUP AND LOOP FUNCTIONS
// ====================

void setup() 
{
  setupPins();
  startup();
  logFile();
  calibrate();
  beginTiming();
}

void loop() 
{
  updateTiming();
  checkSensors();
  // serialLog();
  
  switch(currentState)
  {
    case GROUND_IDLE:
      checkLaunch();
      currentState = POWERED_FLIGHT;
      break;
    case POWERED_FLIGHT:
      updatePID();
      checkBurnout();
      break;
    case UNPOWERED_FLIGHT:
      checkApogee();
      break;
    case BALLISTIC_DESCENT:
      checkChuteStop();
      break;
    case CHUTE_DESCENT:
      checkLanded();
      break;
    case GROUND_SAFE:
      showSafe();
      break;
    case ABORT:
      abort();
      break;
  }

  // logData();
}
