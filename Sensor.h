#ifndef Sensor_h
#define Sensor_h

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "SparkFun_VL53L1X.h"
#include "DataTypes.h"
#include "ParameterTypes.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define INTERRUPT_PIN_MPU 23
#define INTERRUPT_PIN_TOF 34
#define VOLTAGE_PIN 36
#define TRIGGER_PIN_OUT 12
#define TRIGGER_PIN_IN 14
#define GYRO_INT_RAD 2000.0 / 2147483647.0  //TODO Calibrate // 16.4
#define VOLTAGE_SCALE 3.94                  //0.00394
#define MAX_FILTER_SIZE 10
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

class SensorPool {
public:
  SensorPool(MPU6050* _mpu, SFEVL53L1X* _tof);
  void setPointer(ParameterSet* _param, SersorState* _sensor, SystemState* _system);
  void setup();
  bool read();
private:
  MPU6050* mpu;
  SFEVL53L1X* tof;
  SersorState* sensor;
  SystemState* system;
  ParameterSet* paramSet;
  uint8_t fifoBuffer[64];  // FIFO storage buffer
  Quaternion q;            // [w, x, y, z]         quaternion container
  VectorInt16 aa;          // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;      // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;     // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;     // [x, y, z]            gravity vector
  float attitude[3];       // [yaw, pitch, roll]   yaw/pitch/roll container
  int32_t rawGyro[3];
  int32_t rawAccel[3];
  LowPassFilter altitudeFilter;
  LowPassFilter voltageFilter;
  int offset = 0;
};

class Timer {
public:
  Timer();
  void reset();
  int tick();
  int average();
private:
  int time1 = 0;
  int time2 = 0;
  int t[5] = { 1, 1, 1, 1, 1 };
  char tp = 0;
  int dt = 0;
};


#endif