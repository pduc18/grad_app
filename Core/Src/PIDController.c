#include "PIDController.h"

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float Ts) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;


    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;
    pid->output = 0;
}

float PID_Compute(PIDController* pid, float setpoint, float measurement) {
    // Shift error history
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    // New error
    pid->error[0] = setpoint - measurement;

    // PID difference equation (incremental form)
    float delta_output = pid->Kp * (pid->error[0] - pid->error[1])
                       + pid->Ki * pid->Ts * pid->error[0]
                       + pid->Kd / pid->Ts * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

    pid->output += delta_output;

    // Anti-windup
    if (pid->output > VOLTAGE_MAX)
        pid->output = VOLTAGE_MAX;
    else if (pid->output < -VOLTAGE_MAX)
        pid->output = -VOLTAGE_MAX;

    return pid->output;
}
