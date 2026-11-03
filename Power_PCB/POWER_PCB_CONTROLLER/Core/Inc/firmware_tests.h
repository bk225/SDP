#ifndef FIRMWARE_TESTS_H_
#define FIRMWARE_TESTS_H_

#include <stdint.h>
#include <stdbool.h>

/* Set to 1 for MOCK-ONLY tests; 0 for the normal application. */
#ifndef FIRMWARE_SELF_TEST
#define FIRMWARE_SELF_TEST 1
#endif

/* Set to 1 only for on-board hardware verification. */
#ifndef FIRMWARE_HARDWARE_TEST
#define FIRMWARE_HARDWARE_TEST 0
#endif

#if FIRMWARE_SELF_TEST && FIRMWARE_HARDWARE_TEST
#error "Select only one firmware test mode"
#endif

#define TEST_FAILURE_LOG_CAPACITY 24

typedef struct {
    const char *stage;
    const char *field;
    uint32_t line;
    int32_t expected;
    int32_t actual;
} FirmwareTestFailure;

#if FIRMWARE_SELF_TEST
extern volatile uint32_t testChecks;
extern volatile uint32_t testFailures;
extern volatile bool testComplete;
extern FirmwareTestFailure testFailureLog[TEST_FAILURE_LOG_CAPACITY];
#endif

/* Entry point called once in main.c when FIRMWARE_SELF_TEST is 1. */
void runFirmwareTests(void);

/* Production time wrapper, using an injected tick during the tests. */
uint32_t getFirmwareTick(void);

#endif /* FIRMWARE_TESTS_H_ */
