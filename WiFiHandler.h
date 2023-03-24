#ifndef WiFiHandler_h
#define WiFiHandler_h

#include <WiFi.h>
#include "WiFiUdp.h"
#include "ParameterTypes.h"
#include "DataTypes.h"
#include <Adafruit_NeoPixel.h>

#define MSG_TYPE_HANDSHAKE 1
#define MSG_TYPE_REQUEST_PARAMETER 2
#define MSG_TYPE_UPDATE_PARAMETER 3
#define MSG_TYPE_SET_TELEMETRY 4
#define MSG_TYPE_TELEMETRY 10
#define MSG_TYPE_CONTROL 20

#define MSG_STRUCTURE_HEADER 0
#define MSG_STRUCTURE_HANDSHAKE_SIZE 1
#define MSG_STRUCTURE_HANDSHAKE_DATA 2
#define MSG_STRUCTURE_PARAMETER_ID 1
#define MSG_STRUCTURE_PARAMETER_DATA 2

#define MSG_STRUCTURE_CONTROL_ARM 1
#define MSG_STRUCTURE_CONTROL_PITCH 2
#define MSG_STRUCTURE_CONTROL_ROLL 4
#define MSG_STRUCTURE_CONTROL_YAW 6
#define MSG_STRUCTURE_CONTROL_THROTTLE 8

#define MSG_TELEMETRY_ARMED 1
#define MSG_TELEMETRY_CYCLETIME 2

#define SIZE_OF_FLOAT 4
#define SIZE_OF_INT16 2
#define SIZE_OF_INT8 1

#define PARAMETER_DISCONNECT_TIMEOUT 10000
#define PARAMETER_FAILSAFE_TIMEOUT 2000
#define PARAMETER_FAILSAFE_THROTTLE 6400

#define WIFI_LED 2
#define WIFI_LOOP_TIME 10

class WiFiHandler {
public:
  WiFiHandler(WiFiUDP* _UDP, const char* _ssid, uint16_t _port);
  void begin();
  void tick();
  void sendTelemetry();
  void setPointer(ParameterSet* _params, MsgTelemetry* _telemetry, MsgControl* _control, SystemState * _system, Adafruit_NeoPixel * _debugLED);
  bool recieve();
private:
  WiFiUDP* udp;
  Adafruit_NeoPixel * debugLED;
  IPAddress remoteIP;
  ParameterSet* paramSet;
  MsgTelemetry* telemetry;
  MsgControl* control;
  SystemState* system;
  const char* ssid;
  uint16_t port;
  uint16_t remotePort;
  uint16_t cycleCount = 1;
  int lastRecvTime = 0;
};

static void setTelemetryPointer(MsgTelemetry *_telemetry, SersorState *_sensor , MotorState *_motor, SystemState * _system, InputState * _input) {
  _telemetry->Attitude = &(_sensor->Attitude);
  _telemetry->TargetAttitude = &(_input->Angle);
  _telemetry->CycleTime = &(_system->CycleTime);
  _telemetry->MotorTimes = &(_motor->MotorTimes);
  _telemetry->Armed = &(_motor->Armed);
  _telemetry->Voltage = &(_sensor->Voltage);  
}

#endif