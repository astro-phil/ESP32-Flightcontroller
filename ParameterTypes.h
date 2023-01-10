#ifndef ParameterTypes_h
#define ParameterTypes_h

#define NUM_PARAMETERS_RANGE 50

#define PARAM_SYSTEM_FLIGHTMODE 1
#define PARAM_SYSTEM_UPDATECYCLE 2
#define PARAM_USER_THROTTLE_SENSE 11
#define PARAM_USER_ROLLPITCH_SENSE 12
#define PARAM_USER_YAW_SENSE 13

#define PARAM_PID_YAW_P 21
#define PARAM_PID_YAW_I 22
#define PARAM_PID_YAW_D 23
#define PARAM_PID_YAW_A 24

#define PARAM_PID_PITCH_ROLL_P 25
#define PARAM_PID_PITCH_ROLL_I 26
#define PARAM_PID_PITCH_ROLL_D 27
#define PARAM_PID_PITCH_ROLL_A 28

#define NUM_PARAMETERS_USED 13

typedef struct {
  float parameters[NUM_PARAMETERS_RANGE];
  uint8_t* parameterIndex;
  uint8_t usedParameters;
} ParameterSet;

static uint8_t usedIndex[NUM_PARAMETERS_USED] = { 1, 2, 11, 12, 13, 21, 22, 23, 24, 25, 26, 27, 28 };

static void setParameterDefaults(ParameterSet* _paramSet, uint8_t* _usedIndex) {
  _paramSet->usedParameters = NUM_PARAMETERS_USED;
  _paramSet->parameterIndex = _usedIndex;
  _paramSet->parameters[PARAM_SYSTEM_UPDATECYCLE] = 3;
  _paramSet->parameters[PARAM_USER_THROTTLE_SENSE] = 0.5;
  _paramSet->parameters[PARAM_USER_ROLLPITCH_SENSE] = 100;
  _paramSet->parameters[PARAM_USER_YAW_SENSE] = 100;
  _paramSet->parameters[PARAM_PID_YAW_P] = 0.01;
  _paramSet->parameters[PARAM_PID_YAW_I] = 0.0;
  _paramSet->parameters[PARAM_PID_YAW_D] = 0.05;
  _paramSet->parameters[PARAM_PID_YAW_A] = 0.05;
  _paramSet->parameters[PARAM_PID_PITCH_ROLL_P] = 0.001;
  _paramSet->parameters[PARAM_PID_PITCH_ROLL_I] = 0.0;
  _paramSet->parameters[PARAM_PID_PITCH_ROLL_D] = 0.005;
  _paramSet->parameters[PARAM_PID_PITCH_ROLL_A] = 0.05;
}

#endif