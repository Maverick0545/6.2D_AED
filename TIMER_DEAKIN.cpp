#include "TIMER_DEAKIN.h"
#include <Arduino.h>

void TIMER_DEAKIN::init() {
    // Enable the TC3 peripheral clock
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;

    // Enable the GCLK for TC3
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3 | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Configure TC3 for 16-bit counter mode with a prescaler of 1024
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_WAVEGEN_MFRQ;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void TIMER_DEAKIN::start(unsigned long duration) {
    // Calculate number of ticks based on time in milliseconds
    // Effective frequency with DIV1024 is 48,000,000 / 1024 = 46875 Hz
    uint16_t ticks = duration * (46875 / 1000);  // Adjust for prescaler of 1024
    TC3->COUNT16.CC[0].reg = ticks; // Set the compare value
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

    // Enable and start the timer
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

bool TIMER_DEAKIN::isComplete() {
    // Check if the timer has reached the compare value
    if (TC3->COUNT16.INTFLAG.bit.MC0) {
        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0; // Clear the interrupt flag
        return true; // Timer has completed
    }
    return false; // Timer is still running
}

void TIMER_DEAKIN::reset() {
    // Stop the timer
    TC3->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}
