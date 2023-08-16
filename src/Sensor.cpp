#include "Sensor.h"

LowPassFilter::LowPassFilter()
{
}

void LowPassFilter::setPointer(ParameterSet *_param, uint8_t _paramID)
{
  paramSet = _param;
  paramID = _paramID;
}

void LowPassFilter::reset()
{
  for (uint8_t i = 0; i < paramSet->Parameters[paramID]; i++)
  {
    store[i] = 0;
  }
}

float LowPassFilter::filter(float x)
{
  index++;
  if (index >= paramSet->Parameters[paramID])
    index = 0;
  store[index] = x;
  float sum = 0;
  for (uint8_t i = 0; i < paramSet->Parameters[paramID]; i++)
  {
    sum += store[i];
  }
  return sum / paramSet->Parameters[paramID];
}

SensorPool::SensorPool(MPU6050 *_mpu)
{
  mpu = _mpu;
}
void SensorPool::setPointer(ParameterSet *_param, SersorState *_sensor, SystemState *_system, Adafruit_NeoPixel *_debugLED)
{
  paramSet = _param;
  sensor = _sensor;
  system = _system;
  debugLED = _debugLED;
  voltageFilter.setPointer(_param, PARAM_FILTER_VOLTAGE);
}

void SensorPool::setup()
{
  Wire.begin();
  Wire.setClock(400000);
  debugLED->setPixelColor(SENSOR_LED, 0, 0, 255);
  debugLED->show();
  mpu->initialize();
  Serial.println(mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  if (mpu->dmpInitialize() != 0)
    while (1)
      ;
  Serial.println("MPU Initialized!");
  delay(2000);
  Serial.println("Calibrating Gyro ...");
  debugLED->setPixelColor(SENSOR_LED, 255, 255, 0);
  debugLED->show();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu->CalibrateAccel(6);
  mpu->CalibrateGyro(6);
  mpu->PrintActiveOffsets();
  mpu->setDMPEnabled(true);
  mpu->dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu->dmpGetQuaternion(&q, fifoBuffer);
  mpu->dmpGetGravity(&gravity, &q);
  mpu->dmpGetQuaternion(&q, fifoBuffer);
  mpu->dmpGetYawPitchRoll(attitude, &q, &gravity);
  sensor->Attitude.x = -attitude[1] * RAD_TO_DEG;
  sensor->Attitude.y = -attitude[2] * RAD_TO_DEG;
  sensor->Attitude.z = -attitude[0] * RAD_TO_DEG;
  sensor->Attitude.w = -attitude[0] * RAD_TO_DEG;
  Serial.println("Sensors initialized!");
  offset = 0;
  voltageFilter.reset();
  system->CycleTime = 0;
  system->DeltaTime = 0;
  debugLED->setPixelColor(SENSOR_LED, 0, 255, 0);
  debugLED->show();
}

void SensorPool::readSensors()
{
  sensor->Voltage = voltageFilter.filter(analogRead(VOLTAGE_PIN) * VOLTAGE_SCALE);
}

void SensorPool::readDMP()
{
  if (mpu->getFIFOCount() < 16)
    return;
  mpu->getFIFOBytes(fifoBuffer + fifoptr * 2, 2);
  fifoptr++;
  if (fifoptr >= 8)
  {
    fifoptr = 0;
    mpu->resetFIFO();
    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetYawPitchRoll(attitude, &q, &gravity);
    if (-(attitude[0] * RAD_TO_DEG + offset) - sensor->Attitude.z > 300)
    {
      offset += 359;
    }
    else if (-(attitude[0] * RAD_TO_DEG + offset) - sensor->Attitude.z < -300)
    {
      offset += -359;
    }
    sensor->Attitude.x = -attitude[1] * RAD_TO_DEG;
    sensor->Attitude.y = -attitude[2] * RAD_TO_DEG;
    sensor->Attitude.z = -(attitude[0] * RAD_TO_DEG + offset);
    sensor->Attitude.w = -attitude[0] * RAD_TO_DEG;
  }
}

void SensorPool::readGyro()
{
  mpu->getMotion6(&rawAcceleration[0], &rawAcceleration[0], &rawAcceleration[0], &rawRotation[0], &rawRotation[1], &rawRotation[2]);
  sensor->AngularVelocity.x = rawRotation[1] * ROTATION_DEG;
  sensor->AngularVelocity.y = -rawRotation[0] * ROTATION_DEG;
  sensor->AngularVelocity.z = rawRotation[2] * ROTATION_DEG;
  sensor->LinearAcceleration.x = rawAcceleration[0] * ACCELERATION_M_S;
  sensor->LinearAcceleration.y = rawAcceleration[1] * ACCELERATION_M_S;
  sensor->LinearAcceleration.z = rawAcceleration[2] * ACCELERATION_M_S;
  sensor->Attitude.x += sensor->AngularVelocity.x * system->DeltaTime;
  sensor->Attitude.y += sensor->AngularVelocity.y * system->DeltaTime;
  sensor->Attitude.z += sensor->AngularVelocity.z * system->DeltaTime;
}

LoopPerformance::LoopPerformance()
{
  time2 = micros();
  time1 = time2;
}
void LoopPerformance::reset()
{
  time2 = micros();
  time1 = time2;
}
long LoopPerformance::tick()
{
  time2 = micros();
  dt = time2 - time1;
  time1 = time2;
  if (tp > 4)
    tp = 0;
  t[tp] = dt;
  tp++;
  return dt;
}
long LoopPerformance::average()
{
  return (t[0] + t[1] + t[2] + t[3] + t[4]) / 5;
}

LoopDelay::LoopDelay(uint8_t _waitTime)
{
  waitTime = _waitTime;
}

void LoopDelay::tick()
{
  time = millis();
}

void LoopDelay::wait()
{
  if (10 - (millis() - time) > 0)
    delay(10 - (millis() - time));
}
