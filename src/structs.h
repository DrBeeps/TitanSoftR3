#ifndef STRUCT_H
#define STRUCT_H

#include <Arduino.h>

#define EARTHG 9.807

enum FlightMode
{
  GROUND_IDLE = 1,
  POWERED_FLIGHT = 2,
  UNPOWERED_FLIGHT = 3,
  BALLISTIC_DESCENT = 4,
  CHUTE_DESCENT = 5,
  GROUND_SAFE = 6,
  ABORT = 7
};
 
struct FalseData
{
  unsigned long time;

  double pitch;
  double tvcY;
  double scaledTVCY;
  double errorY;
};

struct FalseDataSimReady
{
  unsigned long time;

  double pitch;
};

union DataPoint
{
  struct {
    unsigned long time;

    FlightMode state;

    double gX;
    double gY;
    double gZ;

    double aX;
    double aY;
    double aZ;

    double yaw;
    double pitch;
    double roll;

    double tvcZ;
    double tvcY;

    double deviationZ;
    double deviationY;

    double altitude;
    
    double battVoltage;


    int DATA_ERROR;
  } values __attribute__((packed));
  uint8_t raw[128];
};

struct DataPoint
{
  unsigned long time;

  FlightMode state;

  double gX;
  double gY;
  double gZ;

  double aX;
  double aY;
  double aZ;

  double yaw;
  double pitch;
  double roll;

  double tvcZ;
  double tvcY;

  double deviationZ;
  double deviationY;

  double altitude;
    
  double battVoltage;


  int DATA_ERROR;
};

#endif