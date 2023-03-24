#ifndef Motor_h
#define Motor_h

#define MOTOR_PIN_FL 18 //16
#define MOTOR_PIN_FR 17
#define MOTOR_PIN_BL 19 //18
#define MOTOR_PIN_BR 16 //19

#define PWM_MIN 3000 // 3277
#define PWM_RANGE 3054
#define PWM_MAX 6554 // 6554
#define PWM_IDLE 3500

#define PWM_FREQUENZ 50

#include "ParameterTypes.h"
#include "DataTypes.h"


class MotorController {
public:
  MotorController();
  void setPointer( ParameterSet* _params, MotorState* _motor, SystemState * _system);
  void stop();
  void low();
  void update();

private:
  SystemState* system;
  MotorState* motor;
  ParameterSet* paramSet;
};

#endif