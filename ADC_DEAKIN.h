#ifndef ADC_DEAKIN_H
#define ADC_DEAKIN_H

#include <Arduino.h>

class ADC_DEAKIN {
public:
    // Sets up the ADC with the given port, pin, resolution, and mode
    bool setup_ADC(char port, int pinNum, int resolution, int mode);

    // Reads the ADC value and stores it in the provided pointer
    bool read_ADC(int *value);

    // Enables the ADC peripheral
    bool enable_ADC();

    // Disables the ADC peripheral
    bool disable_ADC();

    // Sets the ADC interrupt (not implemented for basic tasks)
    bool setADCInterrupt();
};

#endif
