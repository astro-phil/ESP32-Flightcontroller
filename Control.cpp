#include "Control.h"
#define limit(val, min, max) ((val > max) ? (max) : ((val < min) ? (min) : (val)))

PID::PID(uint8_t _p, uint8_t _i, uint8_t _d, uint8_t _a) {
  idP = _p;
  idI = _i;
  idD = _d;
  idA = _a;
}

void PID::setPointer(ParameterSet* _param, SystemState* _system) {
  paramSet = _param;
  system = _system;
}

void PID::reset() {
  state = 0;
  last_x_mess = 0;
}

float PID::step(float x_mess, float xdot_mess, float x_goal, float xdot_goal) {
  state += (x_goal - x_mess) * system->DeltaTime;
  state = limit(state, -paramSet->Parameters[idA], paramSet->Parameters[idA]);
  return (x_goal - x_mess) * paramSet->Parameters[idP]
         + (xdot_goal - xdot_mess) * paramSet->Parameters[idD] + state * paramSet->Parameters[idI];
}

float PID::step(float x_mess, float xdot_mess, float x_goal) {
  state += (x_goal - x_mess) * system->DeltaTime;
  state = limit(state, -paramSet->Parameters[idA], paramSet->Parameters[idA]);
  return (x_goal - x_mess) * paramSet->Parameters[idP]
         + xdot_mess * paramSet->Parameters[idD] + state * paramSet->Parameters[idI];
}

float PID::step(float x_mess, float x_goal) {
  float xdot = -(x_mess - last_x_mess) / (system->CycleTime + 0.1);
  last_x_mess = x_mess;
  state += (x_goal - x_mess) * system->DeltaTime;
  state = limit(state, -paramSet->Parameters[idA], paramSet->Parameters[idA]);
  return (x_goal - x_mess) * paramSet->Parameters[idP]
         + xdot * paramSet->Parameters[idD] + state * paramSet->Parameters[idI];
}

// --------------------------------------------------------------------------------------------------------------
Integrator::Integrator() {
  state = 0;
}

void Integrator::setPointer(SystemState* _system) {
  system = _system;
}

void Integrator::reset() {
  state = 0;
}

void Integrator::reset(float x) {
  state = x;
}

float Integrator::step(float xdot) {
  state += xdot * system->DeltaTime;
  return state;
}

float Integrator::step(float xdot, float min, float max) {
  state += xdot * system->DeltaTime;
  state = limit(state, min, max);
  return state;
}

// --------------------------------------------------------------------------------------------------------------