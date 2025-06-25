#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "PIDController.h"
#include <float.h>

#define VOLTAGE_MAX 12.0f // Maximum voltage for motor control

typedef struct {
    int32_t encoderValueNow;
    int32_t encoderValuePrev;
    int32_t positionPulse;
    float position;
} Motor_TypeDef;

void Motor_Init(void);
float Motor_GetCurrentPosition_mm(void);
void SetMotorOutput(float voltage);
void Motor_ComputePID(float current_position, float target_position_mm);
float LookupTargetPosition(float x, float y);

#endif // MOTORCONTROLLER_H