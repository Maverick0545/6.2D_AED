#include "GPIO_DEAKIN.h"
#include <Arduino.h>

void GPIO_DEAKIN::pinMode(uint8_t pin, uint8_t mode) {
    // Check if the pin is D0 or D1 (PA11 or PA10)
    if (pin == 0 || pin == 1) {
        // Avoid modifying pins D0 and D1 used by Serial1
        return;
    }
    
    // Configure the GPIO pin direction using SAMD21 register manipulation
    if (mode == OUTPUT) {
        PORT->Group[0].DIRSET.reg = (1 << pin); // Set the pin as output
    } else {
        PORT->Group[0].DIRCLR.reg = (1 << pin); // Set the pin as input
    }
}

void GPIO_DEAKIN::digitalWrite(uint8_t pin, uint8_t val) {
    // Check if the pin is D0 or D1 (PA11 or PA10)
    if (pin == 0 || pin == 1) {
        // Avoid modifying pins D0 and D1 used by Serial1
        return;
    }
    
    // Set pin state directly via SAMD21 register manipulation
    if (val == HIGH) {
        PORT->Group[0].OUTSET.reg = (1 << pin); // Set pin high
    } else {
        PORT->Group[0].OUTCLR.reg = (1 << pin); // Set pin low
    }
}

void GPIO_DEAKIN::ConfigurePins(uint8_t pinMask, uint8_t mode) {
    // Mask out D0 and D1 (PA11 and PA10) to avoid altering Serial1 configuration
    pinMask &= ~((1 << 0) | (1 << 1));  // Exclude pins D0 (PA11) and D1 (PA10)
    
    // Configure multiple pins at once using a pin mask
    if (mode == OUTPUT) {
        PORT->Group[0].DIRSET.reg = pinMask;
    } else {
        PORT->Group[0].DIRCLR.reg = pinMask;
    }
}
