#ifndef DataTypes_h
#define DataTypes_h
#include <stdint.h>

typedef struct {
  float x;
  float y;
  float z;
} FVector3;

typedef struct {
  float x;
  float y;
  float z;
  float w;
} FVector4;

typedef struct {
  uint16_t fl;
  uint16_t fr;
  uint16_t bl;
  uint16_t br;
} IMotor4;

typedef struct {
  FVector4 Attitude;
  FVector3 Error;
  FVector3 AngularVelocity;
  FVector3 LinearAcceleration;
  uint16_t Voltage; 
} SersorState;

typedef struct {
  bool Telemetry;
  uint8_t ConnectionState;
  uint8_t Flightmode;
  uint16_t CycleTime;
  float DeltaTime;
} SystemState;

typedef struct {
  FVector4 Control;
  IMotor4 MotorTimes;
  bool Armed;
} MotorState;

typedef struct {
  FVector3 AngularVelocity;
  FVector3 Angle;
  bool Arm; 
} InputState;

typedef struct {
  FVector4 *Attitude;
  FVector3 *TargetAttitude;
  IMotor4 *MotorTimes;
  uint16_t *CycleTime;
  bool *Armed;
  uint16_t *Voltage;
} MsgTelemetry;

typedef struct {
  uint8_t Arm;
  int16_t Yaw;
  int16_t Pitch;
  int16_t Roll;
  int16_t Throttle;
} MsgControl;

#endif