#include "ADC_DEAKIN.h"
#include <Arduino.h>

bool ADC_DEAKIN::setup_ADC(char Port, int Pin_Num, int bits, int mode) {
    // Reset the ADC
    ADC->CTRLA.bit.SWRST = 1;
    while (ADC->CTRLA.bit.SWRST == 1); // Wait for reset to complete

    // Set resolution (bits)
    if (bits == 10) {
        ADC->CTRLB.bit.RESSEL = 0x0; // Set 10-bit resolution
    }

    // Set reference voltage to 3.3V
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // VDDANA / 2 reference

    // Configure input (Pin)
    // A3 corresponds to ADC input pin 5 on Arduino Nano 33 IoT
    ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN5; // A3 corresponds to PIN5

    // Set mode (single-shot or continuous)
    if (mode == 0) {
        ADC->CTRLB.bit.FREERUN = 0; // Single-shot mode
    } else {
        ADC->CTRLB.bit.FREERUN = 1; // Continuous conversion mode
    }

    // Enable ADC
    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY); // Wait for synchronization

    return true;
}

bool ADC_DEAKIN::read_ADC(int *sample) {
    // Start conversion
    ADC->SWTRIG.bit.START = 1;
    while (ADC->INTFLAG.bit.RESRDY == 0); // Wait for conversion to complete

    // Read result
    *sample = ADC->RESULT.reg;
    return true;
}

bool ADC_DEAKIN::enable_ADC() {
    ADC->CTRLA.bit.ENABLE = 1; // Enable ADC
    while (ADC->STATUS.bit.SYNCBUSY); // Wait for synchronization
    return true;
}

bool ADC_DEAKIN::disable_ADC() {
    ADC->CTRLA.bit.ENABLE = 0; // Disable ADC
    while (ADC->STATUS.bit.SYNCBUSY); // Wait for synchronization
    return true;
}
