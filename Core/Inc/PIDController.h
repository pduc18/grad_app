#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <math.h>

#define VOLTAGE_MAX 12.0f // Maximum voltage for the motor

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Ts;

    float prev_error;
    float integral;
    float output;
} PIDController;

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float Ts);
float PID_Compute(PIDController* pid, float setpoint, float measurement);

#endif
