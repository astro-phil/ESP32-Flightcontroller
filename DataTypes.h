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
  FVector3 AngularVelocity;
  uint16_t Altitude;
  uint16_t Voltage; 
} SersorState;

typedef struct {
  bool connected;
  uint8_t flightmode;
  uint16_t CycleTime;
} SystemState;

typedef struct {
  FVector4 Control;
  IMotor4 MotorTimes;
  bool armed;
} MotorState;

typedef struct {
  FVector3 AngularVelocity;
  FVector3 Angle;
  bool arm; 
  uint16_t Altitude;
} InputState;

typedef struct {
  FVector4 *Attitude;
  uint16_t *Altitude;
  IMotor4 *MotorTimes;
  uint16_t *CycleTime;
  bool *armed;
  uint16_t *Voltage;
} MsgTelemetry;

typedef struct {
  uint8_t arm;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  int16_t throttle;
} MsgControl;

#endif