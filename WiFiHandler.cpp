#include "WiFiHandler.h"

WiFiHandler::WiFiHandler(WiFiUDP *_UDP, const char *_ssid, uint16_t _port) {
  ssid = _ssid;
  udp = _UDP;
  port = _port;
}

void WiFiHandler::begin() {
  WiFi.softAP(ssid);
  udp->begin(WiFi.localIP(), port);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Local IP is ");
  Serial.println(IP);
  Serial.print("Listening on Port:");
  Serial.println(port);
}

void WiFiHandler::setPointer(ParameterSet *_params, MsgTelemetry *_telemetry, MsgControl *_control, SystemState * _system) {
  paramSet = _params;
  telemetry = _telemetry;
  control = _control;
  system = _system;
}

void WiFiHandler::tick() {
  if(connected && millis()-lastRecvTime > PARAMETER_DISCONNECT_TIMEOUT){
    connected = false;
    system->connected = false;
    runTelemetry = false;
    control->arm = 0;
    Serial.println("Connection Lost!");
  }
}


bool WiFiHandler::recieve() {
  int packetSize = udp->parsePacket();
  if (packetSize) {
    uint8_t msg[64];
    uint8_t buffer[64];
    udp->read(msg, packetSize);
    lastRecvTime = millis();
    switch (msg[MSG_STRUCTURE_HEADER]) {
      case MSG_TYPE_CONTROL:
        memcpy(&control->arm, msg + MSG_STRUCTURE_CONTROL_ARM, SIZE_OF_INT8);
        memcpy(&control->pitch, msg + MSG_STRUCTURE_CONTROL_PITCH, SIZE_OF_INT16);
        memcpy(&control->roll, msg + MSG_STRUCTURE_CONTROL_ROLL, SIZE_OF_INT16);
        memcpy(&control->yaw, msg + MSG_STRUCTURE_CONTROL_YAW, SIZE_OF_INT16);
        memcpy(&control->throttle, msg + MSG_STRUCTURE_CONTROL_THROTTLE, SIZE_OF_INT16);
        break;
      case MSG_TYPE_HANDSHAKE:
        Serial.println("Recieved Handshake!");
        runTelemetry = false;
        remoteIP = udp->remoteIP();
        remotePort = udp->remotePort();
        buffer[MSG_STRUCTURE_HEADER] = MSG_TYPE_HANDSHAKE;
        buffer[MSG_STRUCTURE_HANDSHAKE_SIZE] = paramSet->usedParameters;
        memcpy(buffer + MSG_STRUCTURE_HANDSHAKE_DATA, paramSet->parameterIndex, sizeof(uint8_t) * (paramSet->usedParameters));
        Serial.print("Send ParamaterSet to ");
        Serial.println(remoteIP);
        udp->beginPacket(remoteIP, remotePort);
        udp->write(buffer, 2 + paramSet->usedParameters);
        udp->endPacket();
        break;
      case MSG_TYPE_REQUEST_PARAMETER:
        Serial.println("Recieved Parameter request!");
        buffer[MSG_STRUCTURE_HEADER] = MSG_TYPE_REQUEST_PARAMETER;
        buffer[MSG_STRUCTURE_PARAMETER_ID] = msg[MSG_STRUCTURE_PARAMETER_ID];
        memcpy(buffer + MSG_STRUCTURE_PARAMETER_DATA, &paramSet->parameters[msg[MSG_STRUCTURE_PARAMETER_ID]], SIZE_OF_FLOAT);
        Serial.printf("Sent Parameter:{%i} = {%f}\n", msg[MSG_STRUCTURE_PARAMETER_ID], paramSet->parameters[msg[MSG_STRUCTURE_PARAMETER_ID]]);
        udp->beginPacket(remoteIP, remotePort);
        udp->write(buffer, 2 + SIZE_OF_FLOAT);
        udp->endPacket();
        connected = true;
        system->connected = true;
        break;
      case MSG_TYPE_UPDATE_PARAMETER:
        Serial.printf("Updated Parameter:{%i} = {%f}", msg[MSG_STRUCTURE_PARAMETER_ID], paramSet->parameters[msg[MSG_STRUCTURE_PARAMETER_ID]]);
        memcpy(&paramSet->parameters[msg[MSG_STRUCTURE_PARAMETER_ID]], msg + MSG_STRUCTURE_PARAMETER_DATA, SIZE_OF_FLOAT);
        Serial.printf(" <- {%f}\n", paramSet->parameters[msg[MSG_STRUCTURE_PARAMETER_ID]]);
        udp->beginPacket(remoteIP, remotePort);
        udp->write(msg, 2 + SIZE_OF_FLOAT);
        udp->endPacket();
        break;
      case MSG_TYPE_SET_TELEMETRY:
        runTelemetry = msg[1] != 0;
        udp->beginPacket(remoteIP, remotePort);
        udp->write(msg, 2);
        udp->endPacket();
        break;
    }
    return true;
  }
  return false;
}

void WiFiHandler::sendTelemetry() {
  if (connected && runTelemetry) {
    cycleCount++;
    if (cycleCount > paramSet->parameters[PARAM_SYSTEM_UPDATECYCLE]) {
      uint8_t outBuffer[27];
      uint8_t messageIndex = 0;
      int16_t aAttitude[] = {telemetry->Attitude->x*160,telemetry->Attitude->y*160,telemetry->Attitude->w*160};
      int16_t aTargetAttitude[] = {telemetry->TargetAttitude->x*160,telemetry->TargetAttitude->y*160,telemetry->TargetAttitude->z*160};
      uint16_t aMotor[] = {telemetry->MotorTimes->fl,telemetry->MotorTimes->fr,telemetry->MotorTimes->bl,telemetry->MotorTimes->br};
      outBuffer[MSG_STRUCTURE_HEADER] = MSG_TYPE_TELEMETRY;
      outBuffer[MSG_TELEMETRY_CYCLETIME] = *(telemetry->CycleTime);
      outBuffer[MSG_TELEMETRY_ARMED] = *(telemetry->armed);
      messageIndex += 3;
      memcpy(outBuffer + messageIndex, aAttitude , SIZE_OF_INT16 * 3);
      messageIndex +=  SIZE_OF_INT16 * 3;
      memcpy(outBuffer + messageIndex, aMotor , SIZE_OF_INT16 * 4);
      messageIndex +=  SIZE_OF_INT16 * 4;
      memcpy(outBuffer + messageIndex, telemetry->Altitude, SIZE_OF_INT16);
      messageIndex +=  SIZE_OF_INT16;
      memcpy(outBuffer + messageIndex, telemetry->Voltage, SIZE_OF_INT16);
      messageIndex +=  SIZE_OF_INT16;
      memcpy(outBuffer + messageIndex, aTargetAttitude , SIZE_OF_INT16 * 3);
      messageIndex +=  SIZE_OF_INT16 * 3;
      udp->beginPacket(remoteIP, remotePort);
      udp->write(outBuffer, 27);
      udp->endPacket();
      cycleCount = 1;
    }
  }
}