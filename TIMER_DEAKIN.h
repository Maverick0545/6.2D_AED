#ifndef TIMER_DEAKIN_H
#define TIMER_DEAKIN_H

#include <Arduino.h>

class TIMER_DEAKIN {
public:
    void init();
    void start(unsigned long duration);
    bool isComplete();
    void reset();

private:
    unsigned long startTime;  // Store the time when the timer starts
    unsigned long duration;    // Store the duration for the timer
};

#endif
