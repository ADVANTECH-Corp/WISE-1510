#ifndef MBED_MIC_H
#define MBED_MIC_H

#include "mbed.h"

class MAX4466 {
    
public:

    MAX4466(PinName pin);
    
    void led_array(float x);
    
    void volume_indicator();
    
    float calibration();
    
    float sound_level();

    double sound();

protected:
    AnalogIn    _pin;
    BusOut      _led1, _led2, _led3, _led4;
    float       _value;
    float       _sum;
    float       _average;
    int         _count;
    float       _sample;
    Timer       _t;
    Timer       _t1;
};

#endif