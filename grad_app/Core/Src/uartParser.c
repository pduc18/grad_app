// uart_parser.c
#include "uartParser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define DMA_BUFFER_SIZE 128
#define TEMP_LINE_BUFFER_SIZE 64
#define M_PI 3.14159265358979323846
#define dy 500 // mm, distance from lidar to transmitter

static float distance = 0.0f;
static float angle = 0.0f;
static float Dtf_value = 0.0f;
static float Dmis_value = 0.0f;

// DMA buffer for UART data
extern uint8_t uart_dma_buffer[DMA_BUFFER_SIZE];

// Last index of the DMA buffer to process
static uint16_t last_index = 0;

// Temporary line buffer for processing incoming data
static char temp_line_buffer[TEMP_LINE_BUFFER_SIZE];

// Function to process the UART data received via DMA
void UART_ProcessDMAData(uint16_t current_write_index) {
    static uint16_t line_len = 0;

    while (last_index != current_write_index) {
        char c = (char)uart_dma_buffer[last_index];
        last_index = (last_index + 1) % DMA_BUFFER_SIZE;

        if (line_len < TEMP_LINE_BUFFER_SIZE - 1) {
            temp_line_buffer[line_len++] = c;
        } else {
            // Buffer overflow - reset and start over
            line_len = 0;
            continue;
        }

        if (c == '\n') {
            // Process the line as before...
            temp_line_buffer[line_len - 1] = '\0';
            
            float temp_distance = 0.0f, temp_angle = 0.0f;
            if (sscanf(temp_line_buffer, "#%f,%f", &temp_distance, &temp_angle) == 2) {
                // Validate data ranges
                if (temp_distance >= 0.0f && temp_distance <= 10.0f && 
                    temp_angle >= -180.0f && temp_angle <= 180.0f) {
                    angle = (temp_angle + 43) * M_PI / 180.0f;
                    Dtf_value = temp_distance * cosf(angle) * 1000.0f;
                    Dmis_value = temp_distance * sinf(angle) * 1000.0f;
                }
            }
            line_len = 0;
        }
    }
}


float getDtf() {
    return Dtf_value;
}

float getDmis() {
    return fabs(Dmis_value - dy); // Adjust Dmis_value based on dy
}
