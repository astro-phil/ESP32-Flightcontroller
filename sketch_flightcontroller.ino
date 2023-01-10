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
PID yawPID(PARAM_PID_YAW_P, PARAM_PID_YAW_I, PARAM_PID_YAW_D, PARAM_PID_YAW_A);
PID pitchPID(PARAM_PID_PITCH_ROLL_P, PARAM_PID_PITCH_ROLL_I, PARAM_PID_PITCH_ROLL_D, PARAM_PID_PITCH_ROLL_A);
PID rollPID(PARAM_PID_PITCH_ROLL_P, PARAM_PID_PITCH_ROLL_I, PARAM_PID_PITCH_ROLL_D, PARAM_PID_PITCH_ROLL_A);

// defifne two tasks for Blink & AnalogRead
void WifiLoop(void *pvParameters);

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
}

void resetControl(){
  yawI.reset(sensorStt.Attitude.z);
  pitchI.reset(sensorStt.Attitude.x);
  rollI.reset(sensorStt.Attitude.y);
  yawPID.reset();
  pitchPID.reset();
  rollPID.reset();
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
  yawPID.setPointer(&paramSet, &systemStt);
  pitchPID.setPointer(&paramSet, &systemStt);
  rollPID.setPointer(&paramSet, &systemStt);
  sensPool.setPointer(&sensorStt, &systemStt);
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
  setTelemetryPointer(&telemetry, &sensorStt, &motorStt, &systemStt);
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
    if (inputStt.arm != motorStt.armed){
      inputStt.arm = motorStt.armed;
      resetControl();
    }
    motorStt.Control.w = control.throttle / 32000.0 * paramSet.parameters[PARAM_USER_THROTTLE_SENSE];
    inputStt.AngularVelocity.x = control.pitch / 32000.0 * paramSet.parameters[PARAM_USER_ROLLPITCH_SENSE];
    inputStt.AngularVelocity.y = control.roll / 32000.0 * paramSet.parameters[PARAM_USER_ROLLPITCH_SENSE];
    inputStt.AngularVelocity.z = control.yaw / 32000.0 * paramSet.parameters[PARAM_USER_YAW_SENSE];
    inputStt.Angle.x = pitchI.step(inputStt.AngularVelocity.x);
    inputStt.Angle.y = rollI.step(inputStt.AngularVelocity.y);
    inputStt.Angle.z = yawI.step(inputStt.AngularVelocity.z);
    motorStt.Control.x = pitchPID.step(sensorStt.Attitude.x, sensorStt.AngularVelocity.x, inputStt.Angle.x, inputStt.AngularVelocity.x);
    motorStt.Control.y = rollPID.step(sensorStt.Attitude.y, sensorStt.AngularVelocity.y, inputStt.Angle.y, inputStt.AngularVelocity.y);
    motorStt.Control.z = yawPID.step(sensorStt.Attitude.z, sensorStt.AngularVelocity.z, inputStt.Angle.z, inputStt.AngularVelocity.z);
    //Serial.printf("pitch %f:%f ; roll %f:%f ; yaw %f:%f \n", sensorStt.AngularVelocity.x, sensorStt.Attitude.x,  sensorStt.AngularVelocity.y, sensorStt.Attitude.y,  sensorStt.AngularVelocity.z, sensorStt.Attitude.z);
    //Serial.printf("pitch %f:%f ; roll %f:%f ; yaw %f:%f \n", inputStt.AngularVelocity.x, inputStt.Angle.x,  inputStt.AngularVelocity.y, inputStt.Angle.y,  inputStt.AngularVelocity.z, inputStt.Angle.z);

    //Serial.println(digitalRead(TRIGGER_PIN_IN));
    motor.update();

    timer.tick();
    systemStt.CycleTime = timer.average();
  };
}