#include "PIDController.h"

void PID_Init(PIDController* pid, float Kp, float Ki, float Kd, float Ts) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;

    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Compute(PIDController* pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error * pid->Ts;
    float derivative = (error - pid->prev_error) / pid->Ts;

    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;

    // Anti-windup
    if (pid->output > VOLTAGE_MAX)
        pid->output = VOLTAGE_MAX;
    else if (pid->output < -VOLTAGE_MAX)
        pid->output = -VOLTAGE_MAX;

    return pid->output;
}

// //Digital PID controller implementation using incremental form
// float PID_Compute(PIDController* pid, float setpoint, float measurement) {
//     // Shift error history
//     pid->error[2] = pid->error[1];
//     pid->error[1] = pid->error[0];

//     // New error
//     pid->error[0] = setpoint - measurement;

//     // PID difference equation (incremental form)
//     float delta_output = pid->Kp * (pid->error[0] - pid->error[1])
//                        + pid->Ki * pid->Ts * pid->error[0]
//                        + pid->Kd / pid->Ts * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

//     pid->output += delta_output;

//     // Anti-windup
//     if (pid->output > VOLTAGE_MAX)
//         pid->output = VOLTAGE_MAX;
//     else if (pid->output < -VOLTAGE_MAX)
//         pid->output = -VOLTAGE_MAX;

//     return pid->output;
// }