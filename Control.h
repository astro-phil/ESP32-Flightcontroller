#ifndef Control_h
#define Control_h

#include "DataTypes.h"
#include "ParameterTypes.h"

class PID {
public:
  PID(uint8_t _p, uint8_t _i, uint8_t _d, uint8_t _a);
  void setPointer(ParameterSet* _param, SystemState* _system);
  void reset();
  float step(float x_mess, float xdot_mess, float x_goal, float xdot_goal);
  float step(float x_mess, float xdot_mess, float x_goal);
  float step(float x_mess, float x_goal);
private:
  float state = 0;
  float last_x_mess = 0;
  ParameterSet* paramSet;
  SystemState* system;
  uint8_t idP, idD, idI, idA;
};

class Integrator {
public:
  Integrator();
  void setPointer(SystemState* _system);
  void reset();
  void reset(float x);
  float step(float xdot);
private:
  float state = 0;
  SystemState* system;
};
#endif
