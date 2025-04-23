#ifndef CONFIG_H
#define CONFIG_H

#include "stm32g4xx.h"
#include <stdbool.h>

// kalman filter
#define ALTITUDE_SIGMA 1.0f
#define ACCELERATION_SIGMA 1.0f
#define MODEL_SIGMA 1.0f
#define KALMAN_PERIOD 0.1

// Kinematic parameters
#define LAUNCH_VELOCITY 8
#define LAUNCH_ACCEL 20
#define REEFING_ALTITUDE 1250
#define LANDED_VELOCITY 2
#define LANDED_ACCEL 1
#define LANDED_ALTITUDE 30
#define LANDED_TIME 3000

// Flight phase
typedef enum {
	Startup,
	Idle,
	Launched,
	DescendingAfterSeparation,
	DescendingAfterReefing,
	Landed
} FlightPhase;

// Pyros
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	uint32_t fire_time;
	bool firing;
} PyroChannel;

#define CHANNEL_COUNT 2
#define CHANNEL_FIRE_TIME 1000
#define SEPARATION_INDEX 0
#define REEFING_INDEX 1

#endif
