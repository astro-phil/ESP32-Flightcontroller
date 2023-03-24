#ifndef Sensor_h
#define Sensor_h

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "DataTypes.h"
#include "ParameterTypes.h"
#include <Adafruit_NeoPixel.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define INTERRUPT_PIN_MPU 35
#define INTERRUPT_PIN_TOF 34
#define VOLTAGE_PIN 36

#define GYRO_INT_DEG 2000.0 / 2147483647.0
#define ROTATION_DEG 2000.0 / 32767.0
#define ACCELERATION_M_S 2000.0 / 32767.0
#define VOLTAGE_SCALE 3.94  //0.00394
#define MAX_FILTER_SIZE 10
#define TIMER_S 1.0 / 1000000.0

#define SENSOR_LED 1
//#define RAD_TO_DEG 57.29577

class LowPassFilter {
public:
  LowPassFilter();
  void setPointer(ParameterSet* _param, uint8_t _paramID);
  void reset();
  float filter(float x);
private:
  float store[MAX_FILTER_SIZE];
  uint8_t index;
  uint8_t paramID;
  ParameterSet* paramSet;
};

class LoopDelay {
public:
  LoopDelay(uint8_t _waitTime);
  void tick();
  void wait();
private:
  unsigned long time = 0;
  uint8_t waitTime = 0;
};

class LoopPerformance {
public:
  LoopPerformance();
  void reset();
  long tick();
  long average();
private:
  long time1 = 0;
  long time2 = 0;
  long t[5] = { 1, 1, 1, 1, 1 };
  char tp = 0;
  long dt = 0;
};

class SensorPool {
public:
  SensorPool(MPU6050* _mpu);
  void setPointer(ParameterSet* _param, SersorState* _sensor, SystemState* _system, Adafruit_NeoPixel* _debugLED);
  void setup();
  void readDMP();
  void readGyro();
  void readSensors();
private:
  MPU6050* mpu;
  Adafruit_NeoPixel* debugLED;
  SersorState* sensor;
  SystemState* system;
  ParameterSet* paramSet;
  uint8_t fifoBuffer[16];  // FIFO storage buffer
  Quaternion q;            // [w, x, y, z]         quaternion container
  VectorInt16 aa;          // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;      // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;     // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;     // [x, y, z]            gravity vector
  float attitude[3];       // [yaw, pitch, roll]   yaw/pitch/roll container
  int16_t rawRotation[3];
  int16_t rawAcceleration[3];
  LowPassFilter voltageFilter;
  int offset = 0;
  uint8_t fifoptr = 0;
};



#endif