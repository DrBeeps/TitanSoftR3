#ifndef CONSTS_H
#define CONSTS_H

#define LOGIC_REF 3.3

#define DIV_HIGH 200000
#define DIV_LOW 100000

const float DIV_MULT = (LOGIC_REF / 1023) * (DIV_HIGH + DIV_LOW) / DIV_LOW;

#define EARTH_G 9.807

#define CHECK_BIT(var, pos) ((var) & (1<<(pos)))

enum FailureMode
{
    BME_INIT = 0b00000001,
    ACCEL_INIT = 0b00000010,
    GYRO_INIT = 0b00000100,
    BAT_VOLTAGE = 0b00001000
};
 
#endif