#ifndef HARDWARE_TESTS_H_
#define HARDWARE_TESTS_H_

#include <stdint.h>

/* One hardware test at a time; activate only in FIRMWARE_HARDWARE_TEST mode.
 * 1: Servo PWM waveform, ALL SERVOS DISCONNECTED; 50 Hz scope measurements.
 * 2: Motor PWM waveform, ALL MOTOR DRIVERS DISCONNECTED; enables kept LOW.
 * 3: Actual USART3 UART wiring/frame test; NO PWM started.
 * 4: Physically move ONE servo gently around 90 degrees (external 5/6V).
 * 5: Physically run ONE motor ONCE for 200 ms; requires explicit opt-in.
 */
#ifndef FIRMWARE_HW_TEST_KIND
#define FIRMWARE_HW_TEST_KIND 1
#endif

/* Change only after reviewing the bench precautions in README_TESTS.md. */
#ifndef FIRMWARE_HW_ALLOW_MOTOR_ACTUATION
#define FIRMWARE_HW_ALLOW_MOTOR_ACTUATION 0
#endif
#ifndef FIRMWARE_HW_MOTOR_INDEX
#define FIRMWARE_HW_MOTOR_INDEX 0
#endif  /* 0=accelerator, 1=braking, 2=steering */
#ifndef FIRMWARE_HW_SERVO_INDEX
#define FIRMWARE_HW_SERVO_INDEX 0
#endif  /* 0 through 4 */

#if FIRMWARE_HW_TEST_KIND < 1 || FIRMWARE_HW_TEST_KIND > 5
#error "FIRMWARE_HW_TEST_KIND must be in the range 1..5"
#endif
#if FIRMWARE_HW_TEST_KIND == 5 && !FIRMWARE_HW_ALLOW_MOTOR_ACTUATION
#error "Physical motor actuation requires FIRMWARE_HW_ALLOW_MOTOR_ACTUATION 1"
#endif
#if FIRMWARE_HW_MOTOR_INDEX > 2 || FIRMWARE_HW_SERVO_INDEX > 4
#error "Invalid selected motor or servo index"
#endif

extern volatile uint8_t hwTestStage;
extern volatile uint32_t hwTestCycles;
extern volatile uint32_t hwTestUartValid;
extern volatile uint32_t hwTestUartInvalid;
extern volatile uint8_t hwTestLastFrame[14];
extern volatile uint8_t hwTestError;

void startHardwareTests(void);
void serviceHardwareTests(void);

#endif /* HARDWARE_TESTS_H_ */
