/*
 * Software-only STM32 firmware tests.
 * Production motor/UART/safety implementations remain in main.c.
 * This test file must be compiled as part of the CubeIDE project.
 */
#include "main.h"
#include "firmware_types.h"
#include "firmware_tests.h"

/* Production globals/functions exported by main.c, used only to test them. */
extern Motor accelerator;
extern Motor braking;
extern Motor steering;
extern SafetyState safetyState;
extern bool motorAuthorized;

extern uint8_t brake_pwm, steering_pwm, accelerator_pwm;
extern uint8_t brake_enable, steering_enable, accelerator_enable;
extern uint8_t servo_angles[6];
extern uint8_t rxIndex, completedPacket[15];
extern volatile bool packetReady;
extern uint32_t lastValidPacketTime;
extern bool hasReceivedValidPacket;

extern int8_t motorDirection(uint8_t pwmCommand);
extern uint16_t motorCommandToCCR(uint8_t pwmCommand);
extern bool isDirectionReversed(int8_t previousDirection, int8_t requestedDirection);
extern void updateMotor(Motor *motor, uint8_t pwmCommand, uint8_t enableCommand, bool authorized);
extern void updateSafetyState(void);
extern void processReceivedByte(uint8_t byte);
extern bool validatePacket(uint8_t *packet);
extern bool applyPacket(uint8_t *packet);
extern bool isCommunicationProlonged(void);

#if FIRMWARE_SELF_TEST
volatile uint32_t testChecks = 0;
volatile uint32_t testFailures = 0;
volatile bool testComplete = false;
FirmwareTestFailure testFailureLog[TEST_FAILURE_LOG_CAPACITY];

static uint32_t mockTick = 0;
static uint8_t PWMrecord = 100;
static uint8_t ENrecord = 0;
static uint32_t callbackSequence = 0;
static uint32_t lastPWMSequence = 0;
static uint32_t lastEnableSequence = 0;
static uint8_t driverPWMRecords[3] = {100, 100, 100};
static uint8_t driverENRecords[3] = {0, 0, 0};

static void mockSetPWM(uint8_t command);
static void mockSetEnable(uint8_t command);

static Motor testMotor = {
    .state = MOTOR_WAITING,
    .direction = 0,
    .stopTime = 0,
    .setPWM = mockSetPWM,
    .setEnable = mockSetEnable,
    .reverseDelayMs = 2000
};
#endif

/* This tiny time abstraction is also used by normal firmware. */
uint32_t getFirmwareTick(void)
{
#if FIRMWARE_SELF_TEST
    return mockTick;
#else
    return HAL_GetTick();
#endif
}

#if FIRMWARE_SELF_TEST
static void recordCheck(const char *stage, const char *field,
                        int32_t expected, int32_t actual, uint32_t line)
{
    testChecks++;
    if (expected != actual) {
        uint32_t index = testFailures;
        testFailures++;
        if (index < TEST_FAILURE_LOG_CAPACITY) {
            testFailureLog[index].stage = stage;
            testFailureLog[index].field = field;
            testFailureLog[index].line = line;
            testFailureLog[index].expected = expected;
            testFailureLog[index].actual = actual;
        }
    }
}

#define TEST_EQ(stage, field, expected, actual) \
    recordCheck((stage), (field), (int32_t)(expected), (int32_t)(actual), __LINE__)

static void checkMotor(const char *stage, MotorState state,
                       uint8_t pwm, uint8_t enabled, int8_t direction)
{
    TEST_EQ(stage, "state", state, testMotor.state);
    TEST_EQ(stage, "PWM", pwm, PWMrecord);
    TEST_EQ(stage, "enable", enabled, ENrecord);
    TEST_EQ(stage, "direction", direction, testMotor.direction);
}

static void resetTestMotor(uint32_t tick, uint32_t delayMs)
{
    mockTick = tick;
    testMotor.state = MOTOR_WAITING;
    testMotor.direction = 0;
    testMotor.stopTime = 0;
    testMotor.reverseDelayMs = delayMs;
    testMotor.setPWM = mockSetPWM;
    testMotor.setEnable = mockSetEnable;
    PWMrecord = 100;
    ENrecord = 0;
    callbackSequence = 0;
    lastPWMSequence = 0;
    lastEnableSequence = 0;
}

static void mockAccPWM(uint8_t value) { driverPWMRecords[0] = value; }
static void mockAccEnable(uint8_t value) { driverENRecords[0] = value; }
static void mockBrakePWM(uint8_t value) { driverPWMRecords[1] = value; }
static void mockBrakeEnable(uint8_t value) { driverENRecords[1] = value; }
static void mockSteerPWM(uint8_t value) { driverPWMRecords[2] = value; }
static void mockSteerEnable(uint8_t value) { driverENRecords[2] = value; }

static void runMotorHelpersTest(void)
{
    TEST_EQ("map", "dir 0", -1, motorDirection(0));
    TEST_EQ("map", "dir 99", -1, motorDirection(99));
    TEST_EQ("map", "dir 100", 0, motorDirection(100));
    TEST_EQ("map", "dir 101", 1, motorDirection(101));
    TEST_EQ("map", "dir 200", 1, motorDirection(200));
    TEST_EQ("map", "dir 201", 0, motorDirection(201));
    TEST_EQ("map", "CCR 0", 5000, motorCommandToCCR(0));
    TEST_EQ("map", "CCR 50", 2500, motorCommandToCCR(50));
    TEST_EQ("map", "CCR 100", 0, motorCommandToCCR(100));
    TEST_EQ("map", "CCR 150", 2500, motorCommandToCCR(150));
    TEST_EQ("map", "CCR 200", 5000, motorCommandToCCR(200));
    TEST_EQ("map", "CCR invalid", 0, motorCommandToCCR(201));
    TEST_EQ("direction", "same", false, isDirectionReversed(1, 1));
    TEST_EQ("direction", "reverse", true, isDirectionReversed(1, -1));
    TEST_EQ("direction", "initial", false, isDirectionReversed(0, -1));
    TEST_EQ("direction", "neutral", false, isDirectionReversed(1, 0));
}

void runMotorTest(void)
{
    uint32_t savedStopTime;
    resetTestMotor(1000, 2000);

    updateMotor(&testMotor, 150, 1, true);
    checkMotor("start forward 150", MOTOR_RUNNING, 150, 1, 1);
    TEST_EQ("start forward 150", "PWM before enable", true,
            lastPWMSequence < lastEnableSequence);

    updateMotor(&testMotor, 180, 1, true);
    checkMotor("forward PWM change", MOTOR_RUNNING, 180, 1, 1);

    updateMotor(&testMotor, 50, 1, true);
    savedStopTime = testMotor.stopTime;
    checkMotor("reverse request", MOTOR_STOPPING, 100, 0, 1);
    TEST_EQ("reverse request", "saved stop tick", 1000, savedStopTime);

    mockTick = 1100;
    updateMotor(&testMotor, 50, 1, false);
    checkMotor("authorization lost", MOTOR_STOPPING, 100, 0, 1);
    TEST_EQ("authorization lost", "stop time unchanged", savedStopTime, testMotor.stopTime);

    mockTick = 1200;
    updateMotor(&testMotor, 50, 0, true);
    checkMotor("disable while stopping", MOTOR_STOPPING, 100, 0, 1);
    TEST_EQ("disable while stopping", "stop time unchanged", savedStopTime, testMotor.stopTime);

    mockTick = 2999;
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("one ms before delay", MOTOR_STOPPING, 100, 0, 1);

    mockTick = 3000;
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("delay reached", MOTOR_WAITING, 100, 0, 1);

    updateMotor(&testMotor, 50, 1, true);
    checkMotor("run reverse 50", MOTOR_RUNNING, 50, 1, -1);
    TEST_EQ("run reverse 50", "PWM before enable", true,
            lastPWMSequence < lastEnableSequence);

    mockTick = 3100;
    updateMotor(&testMotor, 160, 1, true);
    checkMotor("reverse to forward", MOTOR_STOPPING, 100, 0, -1);
    TEST_EQ("reverse to forward", "stop tick", 3100, testMotor.stopTime);

    mockTick = 5099;
    updateMotor(&testMotor, 160, 1, true);
    checkMotor("forward before delay", MOTOR_STOPPING, 100, 0, -1);

    mockTick = 5100;
    updateMotor(&testMotor, 160, 1, true);
    checkMotor("forward delay reached", MOTOR_WAITING, 100, 0, -1);

    updateMotor(&testMotor, 160, 1, true);
    checkMotor("run forward 160", MOTOR_RUNNING, 160, 1, 1);

    // A neutral command while running must immediately disable the bridge.
    mockTick = 6000;
    updateMotor(&testMotor, 100, 1, true);
    checkMotor("neutral", MOTOR_WAITING, 100, 0, 1);
    TEST_EQ("neutral", "stop tick", 6000, testMotor.stopTime);
    updateMotor(&testMotor, 160, 1, true);
    checkMotor("restart same direction", MOTOR_RUNNING, 160, 1, 1);

    // Changing to reverse after a stop must not bypass the delay.
    mockTick = 6010;
    updateMotor(&testMotor, 50, 0, true);
    checkMotor("disabled command", MOTOR_WAITING, 100, 0, 1);
    TEST_EQ("disabled command", "stop tick", 6010, testMotor.stopTime);
    mockTick = 6020;
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("early restart reverse", MOTOR_WAITING, 100, 0, 1);
    mockTick = 8010;
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("reverse after neutral delay", MOTOR_RUNNING, 50, 1, -1);

    // If a pending reversal is canceled, the previous direction may resume.
    resetTestMotor(9000, 2000);
    updateMotor(&testMotor, 150, 1, true);
    mockTick = 9100;
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("cancel setup", MOTOR_STOPPING, 100, 0, 1);
    mockTick = 9101;
    updateMotor(&testMotor, 150, 1, true);
    checkMotor("cancel reversal", MOTOR_WAITING, 100, 0, 1);
    updateMotor(&testMotor, 150, 1, true);
    checkMotor("resume original direction", MOTOR_RUNNING, 150, 1, 1);

    // Check the other actuator delay (1000 ms).
    resetTestMotor(10000, 1000);
    updateMotor(&testMotor, 150, 1, true);
    updateMotor(&testMotor, 50, 1, true);
    mockTick = 10999;
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("actuator 999 ms", MOTOR_STOPPING, 100, 0, 1);
    mockTick = 11000;
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("actuator 1000 ms", MOTOR_WAITING, 100, 0, 1);

    // The unsigned HAL tick can wrap around and still produce correct elapsed time.
    resetTestMotor(0xFFFFFF00u, 2000);
    updateMotor(&testMotor, 150, 1, true);
    updateMotor(&testMotor, 50, 1, true);
    mockTick = (uint32_t)(0xFFFFFF00u + 1999u);
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("tick rollover 1999", MOTOR_STOPPING, 100, 0, 1);
    mockTick = (uint32_t)(0xFFFFFF00u + 2000u);
    updateMotor(&testMotor, 50, 1, true);
    checkMotor("tick rollover 2000", MOTOR_WAITING, 100, 0, 1);

    resetTestMotor(13000, 2000);
    updateMotor(&testMotor, 150, 1, true);
    updateMotor(&testMotor, 201, 1, true);
    checkMotor("invalid PWM disables", MOTOR_WAITING, 100, 0, 1);
    updateMotor(&testMotor, 150, 2, true);
    checkMotor("invalid enable disables", MOTOR_WAITING, 100, 0, 1);
}

static void computeTestChecksum(uint8_t *packet)
{
    uint16_t sum = 0;
    for (uint8_t i = 2; i < 14; i++) sum += packet[i];
    packet[14] = (uint8_t)sum;
}

static void runProtocolTest(void)
{
    uint8_t packet[15] = {
        0xAA, 0x55, 1, 0, 1, 100, 0, 200, 0, 30, 60, 90, 120, 180, 0
    };
    computeTestChecksum(packet);

    TEST_EQ("packet", "valid", true, validatePacket(packet));
    TEST_EQ("packet", "applied", true, applyPacket(packet));
    TEST_EQ("packet", "brake enable", 1, brake_enable);
    TEST_EQ("packet", "steer enable", 0, steering_enable);
    TEST_EQ("packet", "acc enable", 1, accelerator_enable);
    TEST_EQ("packet", "acc PWM", 200, accelerator_pwm);
    TEST_EQ("packet", "servo 0", 0, servo_angles[0]);
    TEST_EQ("packet", "servo 5", 180, servo_angles[5]);

    packet[0] = 0xAB;
    TEST_EQ("invalid header", "rejected", false, applyPacket(packet));
    packet[0] = 0xAA;
    packet[14] ^= 0x01;
    TEST_EQ("invalid checksum", "rejected", false, applyPacket(packet));
    computeTestChecksum(packet);

    packet[2] = 2; computeTestChecksum(packet);
    TEST_EQ("invalid enable", "rejected", false, applyPacket(packet));
    packet[2] = 1;
    packet[7] = 201; computeTestChecksum(packet);
    TEST_EQ("invalid PWM", "rejected", false, applyPacket(packet));
    packet[7] = 200;
    packet[13] = 181; computeTestChecksum(packet);
    TEST_EQ("invalid angle", "rejected", false, applyPacket(packet));
    TEST_EQ("invalid angle", "old command retained", 200, accelerator_pwm);
    packet[13] = 180; computeTestChecksum(packet);

    // Byte-level receive, including garbage, repeated start byte, and busy slot.
    rxIndex = 0;
    packetReady = false;
    processReceivedByte(0x00);
    TEST_EQ("RX", "skip garbage", 0, rxIndex);
    processReceivedByte(0xAA);
    processReceivedByte(0xAA);
    processReceivedByte(0x55);
    TEST_EQ("RX", "header recognized", 2, rxIndex);
    for (uint8_t i = 2; i < 15; i++) processReceivedByte(packet[i]);
    TEST_EQ("RX", "frame ready", true, packetReady);
    TEST_EQ("RX", "index reset", 0, rxIndex);
    for (uint8_t i = 0; i < 15; i++)
        TEST_EQ("RX", "frame contents", packet[i], completedPacket[i]);

    uint8_t firstPWM = completedPacket[7];
    packet[7] = 150; computeTestChecksum(packet);
    for (uint8_t i = 0; i < 15; i++) processReceivedByte(packet[i]);
    TEST_EQ("RX", "busy frame dropped", firstPWM, completedPacket[7]);
    packetReady = false;
    for (uint8_t i = 0; i < 15; i++) processReceivedByte(packet[i]);
    TEST_EQ("RX", "new frame accepted", 150, completedPacket[7]);
    packetReady = false;
    rxIndex = 0;

    // 250 ms is allowed; 251 ms is timed out in this implementation.
    mockTick = 1000;
    hasReceivedValidPacket = false;
    TEST_EQ("timeout", "no first packet", true, isCommunicationProlonged());
    hasReceivedValidPacket = true;
    lastValidPacketTime = 1000;
    mockTick = 1250;
    TEST_EQ("timeout", "at 250 ms", false, isCommunicationProlonged());
    mockTick = 1251;
    TEST_EQ("timeout", "at 251 ms", true, isCommunicationProlonged());
}

static void resetSafetyTestMotors(void)
{
    accelerator.state = MOTOR_WAITING;
    accelerator.direction = 0;
    accelerator.stopTime = 0;
    accelerator.setPWM = mockAccPWM;
    accelerator.setEnable = mockAccEnable;
    braking.state = MOTOR_WAITING;
    braking.direction = 0;
    braking.stopTime = 0;
    braking.setPWM = mockBrakePWM;
    braking.setEnable = mockBrakeEnable;
    steering.state = MOTOR_WAITING;
    steering.direction = 0;
    steering.stopTime = 0;
    steering.setPWM = mockSteerPWM;
    steering.setEnable = mockSteerEnable;
    for (uint8_t i = 0; i < 3; i++) {
        driverPWMRecords[i] = 100;
        driverENRecords[i] = 0;
    }
}

static void runSafetyTest(void)
{
    resetSafetyTestMotors();
    accelerator_pwm = 160; accelerator_enable = 1;
    brake_pwm = 100; brake_enable = 1;
    steering_pwm = 60; steering_enable = 1;
    mockTick = 20000;
    lastValidPacketTime = 20000;
    hasReceivedValidPacket = true;

    safetyState = SAFETY_DISARMED;
    updateSafetyState();
    TEST_EQ("safety", "disarmed auth", false, motorAuthorized);
    TEST_EQ("safety", "disarmed accel", 0, driverENRecords[0]);
    TEST_EQ("safety", "disarmed steer", 0, driverENRecords[2]);

    safetyState = SAFETY_ARMED;
    updateSafetyState();
    TEST_EQ("safety", "authorized", true, motorAuthorized);
    TEST_EQ("safety", "accel on", 1, driverENRecords[0]);
    TEST_EQ("safety", "accel PWM", 160, driverPWMRecords[0]);
    TEST_EQ("safety", "neutral brake off", 0, driverENRecords[1]);
    TEST_EQ("safety", "steering on", 1, driverENRecords[2]);
    TEST_EQ("safety", "steering PWM", 60, driverPWMRecords[2]);

    mockTick = 20251;
    updateSafetyState();
    TEST_EQ("safety", "timed out auth", false, motorAuthorized);
    TEST_EQ("safety", "timed out accel", 0, driverENRecords[0]);
    TEST_EQ("safety", "timed out steer", 0, driverENRecords[2]);

    // This firmware intentionally recovers automatically on a valid fresh packet.
    mockTick = 20260;
    lastValidPacketTime = 20260;
    updateSafetyState();
    TEST_EQ("safety", "auto-recovery auth", true, motorAuthorized);
    TEST_EQ("safety", "auto-recovery accel", 1, driverENRecords[0]);
    TEST_EQ("safety", "auto-recovery steer", 1, driverENRecords[2]);

    safetyState = SAFETY_FAULT;
    updateSafetyState();
    TEST_EQ("safety", "fault auth", false, motorAuthorized);
    TEST_EQ("safety", "fault accel", 0, driverENRecords[0]);
    TEST_EQ("safety", "fault steer", 0, driverENRecords[2]);

    safetyState = SAFETY_DISARMED;
    motorAuthorized = false;
}

void runFirmwareTests(void)
{
    testComplete = false;
    testChecks = 0;
    testFailures = 0;
    runMotorHelpersTest();
    runMotorTest();
    runProtocolTest();
    runSafetyTest();
    testComplete = true;
}

static void mockSetPWM(uint8_t command)
{
    PWMrecord = command;
    lastPWMSequence = ++callbackSequence;
}

static void mockSetEnable(uint8_t command)
{
    ENrecord = command;
    lastEnableSequence = ++callbackSequence;
}


#endif /* FIRMWARE_SELF_TEST */
