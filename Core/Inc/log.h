#ifndef LOG_H
#define LOG_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// Example LogMessage structure.
// Adjust fields and types as needed.
typedef struct {
    uint32_t time_ms;
    FlightPhase phase;
    float kf_pos;
    float kf_vel;
    float kf_accel;
    float altitude;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float pressure;
    int16_t temp;
    uint32_t apogee;
    bool launched;
    uint32_t landed_time;
} LogMessage;

// The following functions constitute our logging interface.
void log_step(void);
void log_setup(void);
void log_start(void);
void log_stop(void);
void log_add(const LogMessage *data);
void log_print_all(void);
void log_erase(void);

#endif /* LOG_H */
