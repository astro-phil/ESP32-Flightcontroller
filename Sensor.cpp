#include "Sensor.h"
#include "MPU6050_6Axis_MotionApps20.h"

LowPassFilter::LowPassFilter() {
}

void LowPassFilter::setPointer(ParameterSet* _param, uint8_t _paramID) {
  paramSet = _param;
  paramID = _paramID;
}

void LowPassFilter::reset() {
  for (uint8_t i = 0; i < paramSet->parameters[paramID]; i++) {
    store[i] = 0;
  }
}

float LowPassFilter::filter(float x) {
  index++;
  if (index >= paramSet->parameters[paramID]) index = 0;
  store[index] = x;
  float sum = 0;
  for (uint8_t i = 0; i < paramSet->parameters[paramID]; i++) {
    sum += store[i];
  }
  return sum / paramSet->parameters[paramID];
}

SensorPool::SensorPool(MPU6050* _mpu, SFEVL53L1X* _tof) {
  mpu = _mpu;
  tof = _tof;
}
void SensorPool::setPointer(ParameterSet* _param, SersorState* _sensor, SystemState* _system) {
  paramSet = _param;
  sensor = _sensor;
  system = _system;
  altitudeFilter.setPointer(_param, PARAM_FILTER_ALTITUDE);
  voltageFilter.setPointer(_param, PARAM_FILTER_VOLTAGE);
}

void SensorPool::setup() {
  Wire.begin();
  Wire.setClock(400000);
  mpu->initialize();
  Serial.println(mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  if (mpu->dmpInitialize() != 0)
    while (1)
      ;
  Serial.println("MPU Initialized!");
  /*
  if (tof->begin() != 0)
    while (1)
      ;
  Serial.println("TOF Initialized!");
  */
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
  //tof->startRanging();
  Serial.println("Sensors initialized!");
  offset = 0;
  altitudeFilter.reset();
  voltageFilter.reset();
}
bool SensorPool::read() {
  if (!mpu->dmpGetCurrentFIFOPacket(fifoBuffer)) { return false; }
  mpu->dmpGetQuaternion(&q, fifoBuffer);
  mpu->dmpGetGravity(&gravity, &q);
  mpu->dmpGetYawPitchRoll(attitude, &q, &gravity);
  mpu->dmpGetGyro(rawGyro, fifoBuffer);
  sensor->Voltage = voltageFilter.filter(analogRead(VOLTAGE_PIN) * VOLTAGE_SCALE);
  sensor->Attitude.x = -attitude[1] * RAD_TO_DEG;
  sensor->Attitude.y = -attitude[2] * RAD_TO_DEG;
  if (-(attitude[0] * RAD_TO_DEG + offset) - sensor->Attitude.z > 300) {
    offset += 359;
  } else if (-(attitude[0] * RAD_TO_DEG + offset) - sensor->Attitude.z < -300) {
    offset += -359;
  }
  sensor->Attitude.z = -(attitude[0] * RAD_TO_DEG + offset);
  sensor->Attitude.w = -attitude[0] * RAD_TO_DEG;
  sensor->AngularVelocity.x = rawGyro[0] * GYRO_INT_RAD;
  sensor->AngularVelocity.y = rawGyro[1] * GYRO_INT_RAD;
  sensor->AngularVelocity.z = rawGyro[2] * GYRO_INT_RAD;
  sensor->Altitude = 0; // altitudeFilter.filter(tof->getDistance()) * cos(attitude[2]) * cos(attitude[1]) + sin(attitude[2]) * 20;
  return true;
}

Timer::Timer() {
  time2 = millis();
  time1 = time2;
}
void Timer::reset() {
  time2 = millis();
  time1 = time2;
}
int Timer::tick() {
  time2 = millis();
  dt = time2 - time1;
  time1 = time2;
  if (tp > 4) tp = 0;
  t[tp] = dt;
  tp++;
  return dt;
}
int Timer::average() {
  return (t[0] + t[1] + t[2] + t[3] + t[4]) / 5;
}
