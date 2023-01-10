#include "esp32-hal-ledc.h"
#include "Motor.h"

#define limit(val, min, max) ((val > max) ? (max) : ((val < min) ? (min) : (val)))

MotorController::MotorController() {
  ledcSetup(0, PWM_FREQUENZ, 16);
  ledcAttachPin(MOTOR_PIN_FL, 0);
  ledcSetup(1, PWM_FREQUENZ, 16);
  ledcAttachPin(MOTOR_PIN_FR, 1);
  ledcSetup(2, PWM_FREQUENZ, 16);
  ledcAttachPin(MOTOR_PIN_BL, 2);
  ledcSetup(3, PWM_FREQUENZ, 16);
  ledcAttachPin(MOTOR_PIN_BR, 3);
}

void MotorController::setPointer(ParameterSet* _params, MotorState* _motor, SystemState* _system) {
  motor = _motor;
  paramSet = _params;
  system = _system;
}

void MotorController::low() {
  ledcWrite(0, PWM_MIN);
  ledcWrite(1, PWM_MIN);
  ledcWrite(2, PWM_MIN);
  ledcWrite(3, PWM_MIN);
}

void MotorController::stop() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void MotorController::update() {
  uint16_t controlFL, controlFR, controlBL, controlBR;
  float base = motor->Control.w;
  float pitch =  0.707 * motor->Control.x;
  float roll =  0.707 * motor->Control.y;
  float yaw =  0.5 * motor->Control.z;

  controlFL = limit(PWM_IDLE + (base + pitch + roll - yaw) * PWM_RANGE, PWM_IDLE, PWM_MAX);
  controlFR = limit(PWM_IDLE + (base + pitch - roll + yaw) * PWM_RANGE, PWM_IDLE, PWM_MAX);
  controlBL = limit(PWM_IDLE + (base - pitch + roll + yaw) * PWM_RANGE, PWM_IDLE, PWM_MAX);
  controlBR = limit(PWM_IDLE + (base - pitch - roll - yaw) * PWM_RANGE, PWM_IDLE, PWM_MAX);

  motor->MotorTimes.fl = controlFL;
  motor->MotorTimes.fr = controlFR;
  motor->MotorTimes.bl = controlBL;
  motor->MotorTimes.br = controlBR;
  if (motor->armed && system->connected) {
    ledcWrite(0, controlFL);
    ledcWrite(1, controlFR);
    ledcWrite(2, controlBL);
    ledcWrite(3, controlBR);
  } else if (system->connected) {
    low();
  } else {
    stop();
  }
}