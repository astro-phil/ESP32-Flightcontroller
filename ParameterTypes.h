#ifndef ParameterTypes_h
#define ParameterTypes_h

#define NUM_PARAMETERS_RANGE 50

#define PARAM_SYSTEM_FLIGHTMODE 1
#define PARAM_SYSTEM_UPDATECYCLE 2
#define PARAM_FILTER_ALTITUDE 3
#define PARAM_FILTER_VOLTAGE 4
#define PARAM_USER_THROTTLE_SENSE 11
#define PARAM_USER_ROLLPITCH_SENSE 12
#define PARAM_USER_YAW_SENSE 13
#define PARAM_PID_Y_P 21
#define PARAM_PID_Y_I 22
#define PARAM_PID_Y_D 23
#define PARAM_PID_Y_A 24
#define PARAM_PID_YDOT_P 25
#define PARAM_PID_YDOT_I 26
#define PARAM_PID_YDOT_D 27
#define PARAM_PID_YDOT_A 28
#define PARAM_PID_PR_P 31
#define PARAM_PID_PR_I 32
#define PARAM_PID_PR_D 33
#define PARAM_PID_PR_A 34
#define PARAM_PID_PRDOT_P 35
#define PARAM_PID_PRDOT_I 36
#define PARAM_PID_PRDOT_D 37
#define PARAM_PID_PRDOT_A 38
#define PARAM_PID_ALT_P 41
#define PARAM_PID_ALT_I 42
#define PARAM_PID_ALT_D 42
#define PARAM_PID_ALT_A 44

#define NUM_PARAMETERS_USED 27

typedef struct {
  float parameters[NUM_PARAMETERS_RANGE];
  uint8_t* parameterIndex;
  uint8_t usedParameters;
} ParameterSet;

static uint8_t usedIndex[NUM_PARAMETERS_USED] = { 1, 2, 3, 4, 11, 12, 13, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 34, 35, 36, 37, 38, 41, 42, 43, 44 };

static void setParameterDefaults(ParameterSet* _paramSet, uint8_t* _usedIndex) {
  _paramSet->usedParameters = NUM_PARAMETERS_USED;
  _paramSet->parameterIndex = _usedIndex;
  _paramSet->parameters[PARAM_SYSTEM_FLIGHTMODE] = 1;
  _paramSet->parameters[PARAM_SYSTEM_UPDATECYCLE] = 3;
  _paramSet->parameters[PARAM_FILTER_ALTITUDE] = 5;
  _paramSet->parameters[PARAM_FILTER_VOLTAGE] = 10;
  _paramSet->parameters[PARAM_USER_THROTTLE_SENSE] = 0.8;
  _paramSet->parameters[PARAM_USER_ROLLPITCH_SENSE] = 150;
  _paramSet->parameters[PARAM_USER_YAW_SENSE] = 150;
  _paramSet->parameters[PARAM_PID_Y_P] = 0.2;
  _paramSet->parameters[PARAM_PID_Y_I] = 0.1;
  _paramSet->parameters[PARAM_PID_Y_D] = 0.001;
  _paramSet->parameters[PARAM_PID_Y_A] = 20;
  _paramSet->parameters[PARAM_PID_YDOT_P] = 0.1;
  _paramSet->parameters[PARAM_PID_YDOT_I] = 0.1;
  _paramSet->parameters[PARAM_PID_YDOT_D] = 0.0;
  _paramSet->parameters[PARAM_PID_YDOT_A] = 0.05;
  _paramSet->parameters[PARAM_PID_PR_P] = 0.15;
  _paramSet->parameters[PARAM_PID_PR_I] = 0.3;
  _paramSet->parameters[PARAM_PID_PR_D] = 0.02;
  _paramSet->parameters[PARAM_PID_PR_A] = 20;
  _paramSet->parameters[PARAM_PID_PRDOT_P] = 0.01;
  _paramSet->parameters[PARAM_PID_PRDOT_I] = 0.01;
  _paramSet->parameters[PARAM_PID_PRDOT_D] = 0.0;
  _paramSet->parameters[PARAM_PID_PRDOT_A] = 0.05;
  _paramSet->parameters[PARAM_PID_ALT_P] = 0.001;
  _paramSet->parameters[PARAM_PID_ALT_I] = 0.01;
  _paramSet->parameters[PARAM_PID_ALT_D] = 0.0;
  _paramSet->parameters[PARAM_PID_ALT_A] = 100;
}

#endif