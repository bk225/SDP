/*
 * Bench-only, selectable STM32 hardware verification.
 * No software-only tests here. Never used in the production path.
 * NEVER attach motor drivers/servos during modes 1 or 2.
 */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "firmware_types.h"
#include "firmware_tests.h"
#include "hardware_tests.h"

#if FIRMWARE_HARDWARE_TEST
extern uint8_t servo_angles[5];
extern uint8_t rxByte;
extern uint8_t completedPacket[14];
extern volatile bool packetReady;
extern Motor accelerator, braking, steering;

extern void disableMotors(void);
extern void initializeMotorPWM(void);
extern void startMotorPWM(void);
extern void startServoPWM(void);
extern void updateServoPWM(void);
extern void setAcceleratorPWM(uint8_t pwmCommand);
extern void setBrakingPWM(uint8_t pwmCommand);
extern void setSteeringPWM(uint8_t pwmCommand);
extern void updateMotor(Motor *motor, uint8_t pwmCommand,
                        uint8_t enableCommand, bool authorized);
extern bool validatePacket(uint8_t *packet);

volatile uint8_t hwTestStage = 0;
volatile uint32_t hwTestCycles = 0;
volatile uint32_t hwTestUartValid = 0;
volatile uint32_t hwTestUartInvalid = 0;
volatile uint8_t hwTestLastFrame[14] = {0};
volatile uint8_t hwTestError = 0;

#if FIRMWARE_HW_TEST_KIND != 3
static uint32_t lastStageTime = 0;
static uint32_t currentStageDelayMs = 2000;
#endif

#if FIRMWARE_HW_TEST_KIND == 2
static void stopAllMotorPWMs(void)
{
    disableMotors();
    setAcceleratorPWM(100);
    setBrakingPWM(100);
    setSteeringPWM(100);
}


#endif

#if FIRMWARE_HW_TEST_KIND != 3
static void startStage(uint8_t stage)
{
    hwTestStage = stage;
    lastStageTime = HAL_GetTick();
    currentStageDelayMs = 2000;

#if FIRMWARE_HW_TEST_KIND == 1
    /* Scope-only: all five servo outputs are active, no servos attached. */
    for (uint8_t i = 0; i < 5; i++) servo_angles[i] = 90;
    if (stage == 1) servo_angles[0] = 0;
    if (stage == 2) servo_angles[1] = 180;
    if (stage == 3) servo_angles[2] = 45;
    if (stage == 4) servo_angles[3] = 135;
    if (stage == 5) servo_angles[4] = 180;
    updateServoPWM();
#elif FIRMWARE_HW_TEST_KIND == 2
    /* Scope-only: 20 kHz PWM signals, motor enables always LOW. */
    stopAllMotorPWMs();
    if (stage == 1) setAcceleratorPWM(150); /* PA8: 50% */
    if (stage == 2) setBrakingPWM(150);     /* PA10: 50% */
    if (stage == 3) setSteeringPWM(150);    /* PA0: 50% */
    if (stage == 4) setAcceleratorPWM(50);  /* PA9: 50% reverse */
    if (stage == 5) setBrakingPWM(50);      /* PA11: reverse */
    if (stage == 6) setSteeringPWM(50);     /* PA1: reverse */
#elif FIRMWARE_HW_TEST_KIND == 4
    /* Gentle servo-only movement; ONE servo, 85-95 degrees. */
    for (uint8_t i = 0; i < 5; i++) servo_angles[i] = 90;
    if (stage == 1) servo_angles[FIRMWARE_HW_SERVO_INDEX] = 95;
    if (stage == 2) servo_angles[FIRMWARE_HW_SERVO_INDEX] = 85;
    updateServoPWM();
#elif FIRMWARE_HW_TEST_KIND == 5
    /* ONE motor, forward only, one 200 ms pulse after 2 s delay. */
    Motor *selected = (FIRMWARE_HW_MOTOR_INDEX == 0) ? &accelerator :
                      (FIRMWARE_HW_MOTOR_INDEX == 1) ? &braking : &steering;
    if (stage == 1) {
        /* Test-only authorization; production SAFETY_ARMED is never changed. */
        updateMotor(selected, 120, 1, true);
        currentStageDelayMs = 200;
    } else {
        updateMotor(selected, 100, 0, false);
        currentStageDelayMs = (stage == 0) ? 2000 : 0;
    }
#else
    (void)stage;
#endif
}

#endif /* non-UART stage implementation */

void startHardwareTests(void)
{
    disableMotors();
#if FIRMWARE_HW_TEST_KIND == 1 || FIRMWARE_HW_TEST_KIND == 4
    updateServoPWM();
    startServoPWM();
#elif FIRMWARE_HW_TEST_KIND == 2 || FIRMWARE_HW_TEST_KIND == 5
    initializeMotorPWM();
    startMotorPWM();
#elif FIRMWARE_HW_TEST_KIND == 3
    /* The Rx interrupt callback in main.c assembles real 14-byte frames. */
    if (HAL_UART_Receive_IT(&huart3, &rxByte, 1) != HAL_OK) {
        hwTestError = 1;
        Error_Handler();
    }
#endif
#if FIRMWARE_HW_TEST_KIND != 3
    startStage(0);
#endif
}

void serviceHardwareTests(void)
{
#if FIRMWARE_HW_TEST_KIND == 3
    if (packetReady) {
        uint8_t frame[14];
        for (uint8_t i = 0; i < 14; i++) {
            frame[i] = completedPacket[i];
            hwTestLastFrame[i] = frame[i];
        }
        /* Main loop owns packetReady; ISR drops any frame while this is true. */
        packetReady = false;
        const bool valid = validatePacket(frame);
        if (valid) hwTestUartValid++;
        else hwTestUartInvalid++;
        /* Reply: A5 5A 01 = valid, A5 5A 00 = invalid. */
        uint8_t reply[3] = {0xA5, 0x5A, valid ? 1u : 0u};
        if (HAL_UART_Transmit(&huart3, reply, 3, 100) != HAL_OK) {
            hwTestError = 2;
        }
    }
#else
#if FIRMWARE_HW_TEST_KIND == 5
    /* Remain disabled permanently after the single on-pulse; reset to retry. */
    if (hwTestStage == 2) return;
#endif
    if ((uint32_t)(HAL_GetTick() - lastStageTime) >= currentStageDelayMs) {
#if FIRMWARE_HW_TEST_KIND == 1
        const uint8_t last = 5;
#elif FIRMWARE_HW_TEST_KIND == 2
        const uint8_t last = 6;
#elif FIRMWARE_HW_TEST_KIND == 4
        const uint8_t last = 2;
#else
        const uint8_t last = 2;
#endif
        uint8_t next = (hwTestStage >= last) ? 0 : (uint8_t)(hwTestStage + 1);
        if (next == 0) hwTestCycles++;
        startStage(next);
    }
#endif
}
#endif /* FIRMWARE_HARDWARE_TEST */
