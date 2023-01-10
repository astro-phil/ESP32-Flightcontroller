#ifndef Sensor_h
#define Sensor_h

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "SparkFun_VL53L1X.h"
#include "DataTypes.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define INTERRUPT_PIN_MPU 23
#define INTERRUPT_PIN_TOF 34
#define VOLTAGE_PIN 36
#define TRIGGER_PIN_OUT 12
#define TRIGGER_PIN_IN 14
#define GYRO_INT_RAD 2000.0 / 2147483647.0  //TODO Calibrate // 16.4
#define VOLTAGE_SCALE  3.94 //0.00394
//#define RAD_TO_DEG 57.29577 

class SensorPool {
public:
  SensorPool(MPU6050* _mpu, SFEVL53L1X* _tof) {
    mpu = _mpu;
    tof = _tof;
  }
  void setPointer(SersorState* _sensor, SystemState* _system) {
    sensor = _sensor;
    system = _system;
  }
  void setup() {
    Wire.begin();
    Wire.setClock(400000);
    mpu->initialize();
    Serial.println(mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    if (mpu->dmpInitialize() != 0)
      while (1)
        ;
    Serial.println("MPU Initialized!");
    if (tof->begin() != 0)
      while (1)
        ;
    Serial.println("TOF Initialized!");
    delay(2000);
    Serial.println("Calibrating Gyro ...");
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu->setXGyroOffset(220);
    mpu->setYGyroOffset(76);
    mpu->setZGyroOffset(-85);
    mpu->setZAccelOffset(1788);  // 1688 factory default for my test chip
    mpu->CalibrateAccel(6);
    mpu->CalibrateGyro(6);
    mpu->PrintActiveOffsets();
    mpu->setDMPEnabled(true);
    Serial.println("Starting TOF...");
    tof->startRanging();
    Serial.println("Sensors initialized!");
  }
  bool read() {
    if (!mpu->dmpGetCurrentFIFOPacket(fifoBuffer)) { return false; }    
    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetYawPitchRoll(attitude, &q, &gravity);
    mpu->dmpGetGyro(rawGyro, fifoBuffer);
    sensor->Voltage = analogRead(VOLTAGE_PIN)*VOLTAGE_SCALE;
    sensor->Attitude.x = attitude[2] * RAD_TO_DEG;
    sensor->Attitude.y = -attitude[1] * RAD_TO_DEG;
    sensor->Attitude.z = -attitude[0] * RAD_TO_DEG;
    sensor->AngularVelocity.x = rawGyro[0] * GYRO_INT_RAD;
    sensor->AngularVelocity.y = rawGyro[1] * GYRO_INT_RAD;
    sensor->AngularVelocity.z = rawGyro[2] * GYRO_INT_RAD;
    sensor->Altitude = tof->getDistance();
    return true;
  }

private:
  MPU6050* mpu;
  SFEVL53L1X* tof;
  SersorState* sensor;
  SystemState* system;
  uint8_t fifoBuffer[64];  // FIFO storage buffer
  Quaternion q;            // [w, x, y, z]         quaternion container
  VectorInt16 aa;          // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;      // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;     // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;     // [x, y, z]            gravity vector
  float attitude[3];       // [yaw, pitch, roll]   yaw/pitch/roll container
  int32_t rawGyro[3];
  int32_t rawAccel[3];
};

class Timer {
public:
  Timer() {
  }
  void reset() {
    time2 = millis();
    time1 = time2;
  }
  int tick() {
    time2 = millis();
    dt = time2 - time1;
    time1 = time2;
    if (tp > 4) tp = 0;
    t[tp] = dt;
    tp++;
    return dt;
  }
  int average() {
    return (t[0] + t[1] + t[2] + t[3] + t[4]) / 5;
  }
private:
  int time1 = 0;
  int time2 = 0;
  int t[5] = { 1, 1, 1, 1, 1 };
  char tp = 0;
  int dt = 0;
};


#endif