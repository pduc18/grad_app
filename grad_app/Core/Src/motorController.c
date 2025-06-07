#include "motorController.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PULSES_PER_REV     3584.0f
#define DIAMETER_MM        8.0f // Wheel diameter in mm
#define PITCH              1.25 // mm
#define WHEEL_CIRCUMFERENCE_MM 25.12f // Wheel circumference in mm
#define DISTANCE_PER_PULSE 0.007f // mm per pulse

extern UART_HandleTypeDef huart2; // UART for debug output
extern TIM_HandleTypeDef htim1;  // Encoder - TIM1
extern TIM_HandleTypeDef htim3;  // PWM - TIM3 CH1 (PA6)

static PIDController pid;
static float target_position_mm = 0;

// Lookup table for x, y to target position mapping
typedef struct {
    float x;
    float y;
    float target_position_mm;
} PositionMapping;

static const PositionMapping position_table[] = {   // (x, y) -> target_position_mm
    {0.0f, 50.0f, 20.0f},
    {0.0f, 150.0f, 20.0f},
    {0.0f, 250.0f, 40.0f},
    {0.0f, 350.0f, 80.0f},
    {0.0f, 450.0f, 100.0f},
    {0.0f, 550.0f, 120.0f},
    {0.0f, 650.0f, 120.0f},
    // {0.0f, 350.0f, 80.0f},
    // {0.0f, 350.0f, 80.0f},
    // {0.0f, 350.0f, 80.0f},
};
#define TABLE_SIZE (sizeof(position_table) / sizeof(position_table[0]))

Motor_TypeDef motor = {
    .encoderValueNow = 0,
    .encoderValuePrev = 0,
    .positionPulse = 0,
    .position = 0,
};

// Find the closest x, y in the lookup table using Euclidean distance
float LookupTargetPosition(float x, float y) {
    float min_distance = FLT_MAX;
    float target_pos = 0.0f;
    float target_pos_mm = 0.0f;

    for (size_t i = 0; i < TABLE_SIZE; i++) {
        float dx = position_table[i].x - x;
        float dy = position_table[i].y - y;
        float distance = sqrtf(dx * dx + dy * dy);
        if (distance < min_distance) {
            min_distance = distance;
            target_pos = position_table[i].target_position_mm;
        }
    }
    return target_pos;
}

void Motor_Init(void) {
    /*
    * Initialize the motor controller
    * Set up the encoder and PWM
    */
    PID_Init(&pid, 0.8f, 0.001f, 0.005f, 0.001f); // PID parameters
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);  // Make sure PWM is off
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IN1 = 0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // IN2 = 0
}
    
void Motor_SetTargetPosition(float mm) {
    target_position_mm = mm;
}

float Motor_GetCurrentPosition_mm(void) {
    // Get the current encoder value
    motor.encoderValueNow = __HAL_TIM_GET_COUNTER(&htim1);

    // Calculate difference between current and previous encoder values
    int32_t delta = motor.encoderValueNow - motor.encoderValuePrev;

    // Overflow handling
    if (delta > 30000) {
        delta -= 65536;
    } else if (delta < -30000) {
        delta += 65536;
    }

    // Update previous encoder value
    motor.encoderValuePrev = motor.encoderValueNow;

    // Update position in pulses
    motor.positionPulse += delta;

    // Calculate position in mm
    motor.position = motor.positionPulse * PITCH / PULSES_PER_REV;
    return motor.position;
}
int getPWM(void) {
    // Get the current PWM value
    return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
}
void SetMotorOutput(float voltage) {

    // Direction control
    if (voltage > 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN1 = 1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // IN2 = 0
    } else if (voltage < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IN1 = 0
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // IN2 = 1
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Dừng
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    // Set PWM duty cycle
    int pwm = (int)(fabsf(voltage) / VOLTAGE_MAX * 1000.0f);  // Convert to 0-1000 PWM duty
    if (pwm > 1000) pwm = 1000;
    if (pwm > 50 && pwm <200) pwm = 200; // Ensure minimum PWM is 200
    if (pwm <= 50) pwm = 0; // Ensure minimum PWM is 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

void Motor_ComputePID(float current_position, float target_position_mm) {
    float output_voltage = PID_Compute(&pid, target_position_mm, current_position);
    SetMotorOutput(output_voltage);
}