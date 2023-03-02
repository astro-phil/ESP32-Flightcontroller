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

#define SYSTEM_LED 0
#define DEBUG_LED_PIN 23
#define DEBUG_LED_BRIGHTNESS 30

// PWM Pins 16 17 18 19

static WiFiUDP udp;
static MPU6050 mpu;

WiFiHandler wifi(&udp, UDP_SSID, UDP_PORT);
SensorPool sensPool(&mpu);
MotorController motor;
LoopPerformance controlPerformer;
LoopDelay secondaryLoopPerformer(WIFI_LOOP_TIME);

Adafruit_NeoPixel debugLED(3, DEBUG_LED_PIN, NEO_GRB + NEO_KHZ800);

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
void SecondaryLoop(void *pvParameters);

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
  debugLED.setBrightness(DEBUG_LED_BRIGHTNESS);
  debugLED.setPixelColor(0, 255, 0, 0);
  debugLED.setPixelColor(1, 255, 0, 0);
  debugLED.setPixelColor(2, 255, 0, 0);
  debugLED.show();
  setParameterDefaults(&paramSet, usedIndex);
  setTelemetryPointer(&telemetry, &sensorStt, &motorStt, &systemStt, &inputStt);
  wifi.setPointer(&paramSet, &telemetry, &control, &systemStt, &debugLED);
  sensPool.setPointer(&paramSet, &sensorStt, &systemStt, &debugLED);
  motor.setPointer(&paramSet, &motorStt, &systemStt);
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
  delay(500);
  debugLED.setPixelColor(SYSTEM_LED, 255, 255, 0);
  debugLED.show();
  sensPool.setup();
  wifi.begin();
  controlPerformer.reset();
  xTaskCreatePinnedToCore(
    SecondaryLoop, "SecondaryLoop"  // A name just for humans
    ,
    3096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 0);
  debugLED.setPixelColor(SYSTEM_LED, 0, 255, 0);
  debugLED.show();
}

void SecondaryLoop(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    if (!wifi.recieve()) {
      secondaryLoopPerformer.tick();
      wifi.sendTelemetry();
      sensPool.readSensors();
      secondaryLoopPerformer.wait();
    }
    wifi.tick();
  }
}

void loop() {
  sensPool.readDMP();
  sensPool.readGyro();
  motorStt.Armed = control.Arm != 0;
  if (inputStt.Arm != motorStt.Armed) {
    inputStt.Arm = motorStt.Armed;
    resetControl();
  }

  inputStt.AngularVelocity.x = control.Pitch / 32000.0 * paramSet.Parameters[PARAM_USER_ROLLPITCH_SENSE];
  inputStt.AngularVelocity.y = control.Roll / 32000.0 * paramSet.Parameters[PARAM_USER_ROLLPITCH_SENSE];
  inputStt.AngularVelocity.z = control.Yaw / 32000.0 * paramSet.Parameters[PARAM_USER_YAW_SENSE];

  inputStt.Angle.x = pitchI.step(inputStt.AngularVelocity.x);
  inputStt.Angle.y = rollI.step(inputStt.AngularVelocity.y);
  inputStt.Angle.z = yawI.step(inputStt.AngularVelocity.z);

  motorStt.Control.z = yawDotPID.step(sensorStt.AngularVelocity.z, yawPID.step(sensorStt.Attitude.z, inputStt.AngularVelocity.z, inputStt.Angle.z));
  motorStt.Control.x = pitchDotPID.step(sensorStt.AngularVelocity.x, pitchPID.step(sensorStt.Attitude.x, inputStt.AngularVelocity.x, inputStt.Angle.x));
  motorStt.Control.y = rollDotPID.step(sensorStt.AngularVelocity.y, rollPID.step(sensorStt.Attitude.y, inputStt.AngularVelocity.y, inputStt.Angle.y));
  motorStt.Control.w = control.Throttle / 32000.0 * paramSet.Parameters[PARAM_USER_THROTTLE_SENSE];

  //Serial.printf("pitch %f:%f ; roll %f:%f ; yaw %f:%f \n", sensorStt.AngularVelocity.x, sensorStt.Attitude.x,  sensorStt.AngularVelocity.y, sensorStt.Attitude.y,  sensorStt.AngularVelocity.z, sensorStt.Attitude.z);
  //Serial.printf("pitch %f:%f ; roll %f:%f ; yaw %f:%f \n", inputStt.AngularVelocity.x, inputStt.Angle.x,  inputStt.AngularVelocity.y, inputStt.Angle.y,  inputStt.AngularVelocity.z, inputStt.Angle.z);
  motor.update();

  systemStt.CycleTime = controlPerformer.average();
  systemStt.DeltaTime = systemStt.CycleTime * TIMER_S;
  controlPerformer.tick();
}