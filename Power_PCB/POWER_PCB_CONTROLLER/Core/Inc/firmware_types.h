#ifndef FIRMWARE_TYPES_H_
#define FIRMWARE_TYPES_H_

#include <stdint.h>

/* Shared production types, formerly declared in main.c. */
typedef enum {
    SAFETY_DISARMED,
    SAFETY_ARMED,
    SAFETY_FAULT
} SafetyState;

typedef enum {
    MOTOR_RUNNING,
    MOTOR_STOPPING,
    MOTOR_WAITING
} MotorState;

typedef struct {
    MotorState state;
    int8_t direction;
    uint32_t stopTime;
    void (*setPWM)(uint8_t pwmCommand);
    void (*setEnable)(uint8_t enableCommand);
    uint32_t reverseDelayMs;
} Motor;

#endif /* FIRMWARE_TYPES_H_ */
