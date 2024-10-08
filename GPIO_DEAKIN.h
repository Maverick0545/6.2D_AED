#ifndef GPIO_DEAKIN_H
#define GPIO_DEAKIN_H

#include <Arduino.h>

class GPIO_DEAKIN {
public:
    // Sets up pin as input or output
    void pinMode(uint8_t pin, uint8_t mode);
    
    // Sets pin to HIGH or LOW
    void digitalWrite(uint8_t pin, uint8_t val);

    // Configure multiple pins at once using a mask
    void ConfigurePins(uint8_t pinMask, uint8_t mode);
};

#endif
