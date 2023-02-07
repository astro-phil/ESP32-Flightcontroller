/*********
  Phil White
*********/

#include "WiFiHandler.h"
#include "DataTypes.h"
#include "ParameterTypes.h"
#include "Sensor.h"
#include "Motor.h"
#include "Control.h"

#define UDP_PORT 4321
#define UDP_SSID "ESP32-FlightController"

// PWM Pins 16 17 18 19

static WiFiUDP udp;
static MPU6050 mpu;
static SFEVL53L1X tof;

WiFiHandler wifi(&udp, UDP_SSID, UDP_PORT);
SensorPool sensPool(&mpu, &tof);
MotorController motor;

Timer timer;
SystemState systemStt;
SersorState sensorStt;
MotorState motorStt;
MsgTelemetry telemetry;
MsgControl control;
ParameterSet paramSet;
InputState inputStt;

Integrator yawI;
Integrator pitchI;
Integrator rollI;
Integrator altI;

PID yawPID(PARAM_PID_Y_P, PARAM_PID_Y_I, PARAM_PID_Y_D, PARAM_PID_Y_A);
PID yawDotPID(PARAM_PID_YDOT_P, PARAM_PID_YDOT_I, PARAM_PID_YDOT_D, PARAM_PID_YDOT_A);
PID pitchPID(PARAM_PID_PR_P, PARAM_PID_PR_I, PARAM_PID_PR_D, PARAM_PID_PR_A);
PID pitchDotPID(PARAM_PID_PRDOT_P, PARAM_PID_PRDOT_I, PARAM_PID_PRDOT_D, PARAM_PID_PRDOT_A);
PID rollPID(PARAM_PID_PR_P, PARAM_PID_PR_I, PARAM_PID_PR_D, PARAM_PID_PR_A);
PID rollDotPID(PARAM_PID_PRDOT_P, PARAM_PID_PRDOT_I, PARAM_PID_PRDOT_D, PARAM_PID_PRDOT_A);
PID altPID(PARAM_PID_ALT_P, PARAM_PID_ALT_I, PARAM_PID_ALT_D, PARAM_PID_ALT_A);

// defifne two tasks for Blink & AnalogRead
void WifiLoop(void *pvParameters);

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
}

void resetControl() {
  yawI.reset(sensorStt.Attitude.z);
  pitchI.reset(sensorStt.Attitude.x);
  rollI.reset(sensorStt.Attitude.y);
  altI.reset(0);
  yawPID.reset();
  pitchPID.reset();
  rollPID.reset();
  yawDotPID.reset();
  pitchDotPID.reset();
  rollDotPID.reset();
  altPID.reset();
}


void setup() {
  Serial.begin(115200);
  Serial.print("Booting ... \n");
  pinMode(INTERRUPT_PIN_MPU, INPUT);
  pinMode(INTERRUPT_PIN_TOF, INPUT);
  pinMode(TRIGGER_PIN_OUT, OUTPUT);
  pinMode(TRIGGER_PIN_IN, INPUT);
  //pinMode(VOLTAGE_PIN, INPUT);
  digitalWrite(TRIGGER_PIN_OUT, HIGH);
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_MPU), dmpDataReady, RISING);
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_TOF), dmpDataReady, RISING);
  pitchI.setPointer(&systemStt);
  rollI.setPointer(&systemStt);
  yawI.setPointer(&systemStt);
  altI.setPointer(&systemStt);
  altPID.setPointer(&paramSet, &systemStt);
  yawPID.setPointer(&paramSet, &systemStt);
  pitchPID.setPointer(&paramSet, &systemStt);
  rollPID.setPointer(&paramSet, &systemStt);
  yawDotPID.setPointer(&paramSet, &systemStt);
  pitchDotPID.setPointer(&paramSet, &systemStt);
  rollDotPID.setPointer(&paramSet, &systemStt);
  sensPool.setPointer(&paramSet, &sensorStt, &systemStt);
  sensPool.setup();

  wifi.begin();
  motor.setPointer(&paramSet, &motorStt, &systemStt);
  xTaskCreatePinnedToCore(
    WifiLoop, "Wifi"  // A name just for humans
    ,
    3096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 0);
  timer.reset();
}

void WifiLoop(void *pvParameters) {
  (void)pvParameters;
  setParameterDefaults(&paramSet, usedIndex);
  setTelemetryPointer(&telemetry, &sensorStt, &motorStt, &systemStt, &inputStt);
  wifi.setPointer(&paramSet, &telemetry, &control, &systemStt);
  for (;;) {
    if (!wifi.recieve()) {
      wifi.sendTelemetry();
      delay(10);
    }
    wifi.tick();
  }
}

void loop() {

  if (sensPool.read()) {
    motorStt.armed = control.arm != 0;
    if (inputStt.arm != motorStt.armed) {
      inputStt.arm = motorStt.armed;
      resetControl();
    }

    inputStt.AngularVelocity.x = control.pitch / 32000.0 * paramSet.parameters[PARAM_USER_ROLLPITCH_SENSE];
    inputStt.AngularVelocity.y = control.roll / 32000.0 * paramSet.parameters[PARAM_USER_ROLLPITCH_SENSE];
    inputStt.AngularVelocity.z = control.yaw / 32000.0 * paramSet.parameters[PARAM_USER_YAW_SENSE];
    inputStt.Altitude = altI.step(control.throttle / 32000.0 * paramSet.parameters[PARAM_USER_THROTTLE_SENSE], 0, 2000);
    inputStt.Angle.x = pitchI.step(inputStt.AngularVelocity.x);
    inputStt.Angle.y = rollI.step(inputStt.AngularVelocity.y);
    inputStt.Angle.z = yawI.step(inputStt.AngularVelocity.z);

    motorStt.Control.z = yawDotPID.step(sensorStt.AngularVelocity.z, yawPID.step(sensorStt.Attitude.z, inputStt.AngularVelocity.z, inputStt.Angle.z));
    motorStt.Control.x = pitchDotPID.step(sensorStt.AngularVelocity.x, pitchPID.step(sensorStt.Attitude.x, inputStt.AngularVelocity.x, inputStt.Angle.x));
    motorStt.Control.y = rollDotPID.step(sensorStt.AngularVelocity.y, rollPID.step(sensorStt.Attitude.y, inputStt.AngularVelocity.y, inputStt.Angle.y));
    motorStt.Control.w = control.throttle / 32000.0 * paramSet.parameters[PARAM_USER_THROTTLE_SENSE];

    //motorStt.Control.w = altPID.step(sensorStt.Altitude,inputStt.Altitude);
    //motorStt.Control.z = yawPID.step(sensorStt.Attitude.z, inputStt.Angle.z) + yawDotPID.step(sensorStt.AngularVelocity.z,inputStt.AngularVelocity.z);
    //motorStt.Control.x = pitchPID.step(sensorStt.Attitude.x, inputStt.Angle.x) + pitchDotPID.step(sensorStt.AngularVelocity.x,inputStt.AngularVelocity.x);
    //motorStt.Control.y = rollPID.step(sensorStt.Attitude.y, inputStt.Angle.y) + rollDotPID.step(sensorStt.AngularVelocity.y,inputStt.AngularVelocity.y);
    //Serial.printf("pitch %f:%f ; roll %f:%f ; yaw %f:%f \n", sensorStt.AngularVelocity.x, sensorStt.Attitude.x,  sensorStt.AngularVelocity.y, sensorStt.Attitude.y,  sensorStt.AngularVelocity.z, sensorStt.Attitude.z);
    //Serial.printf("pitch %f:%f ; roll %f:%f ; yaw %f:%f \n", inputStt.AngularVelocity.x, inputStt.Angle.x,  inputStt.AngularVelocity.y, inputStt.Angle.y,  inputStt.AngularVelocity.z, inputStt.Angle.z);

    //Serial.println(digitalRead(TRIGGER_PIN_IN));
    motor.update();

    timer.tick();
    systemStt.CycleTime = timer.average();
  };
}