#ifndef LOG_H
#define LOG_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

#pragma pack(push,1)
typedef struct {
    uint32_t     time_ms;    //  4
    FlightPhase  phase;      //  4
    float        kf_pos;     //  4
    float        kf_vel;     //  4
    float        kf_accel;   //  4
    float        altitude;   //  4
    float        accel_x;    //  4
    float        accel_y;    //  4
    float        accel_z;    //  4
    float        gyro_x;     //  4
    float        gyro_y;     //  4
    float        gyro_z;     //  4
    float        pressure;   //  4
    int16_t      temp;       //  2
} LogMessage;
#pragma pack(pop)

void log_step(void);
void log_setup(void);
void log_start(void);
void log_stop(void);
void log_add(const LogMessage *data);
void log_print_all(void);
void log_erase(void);

#endif /* LOG_H */
