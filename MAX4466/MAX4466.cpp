#include "mbed.h"
#include "MAX4466.h"

#define MAX4466_DEBUG 1

#if MAX4466_DEBUG
//extern Serial serial;
extern int node_printf_to_serial(const char * format, ...); 
#endif

MAX4466::MAX4466(PinName pin): _pin(pin), _led1(NC), _led2(NC), _led3(NC), _led4(NC) {
    calibration();
}

//CALIBRATION FUNCTION:
//Find average sound level at the intialization of the microphone
//Use this in the calculation of the LED indication array values
float MAX4466::calibration() {
    
    _t.start();
    _t1.start();
    float peakToPeak=0, signalMax=0, signalMin=1024;

    while (_t1.read()<1) {
        while (_t.read_ms()<50) {
            _sample=_pin.read();

            if (_sample<1024) {
                if (_sample>signalMax)
                    signalMax=_sample;

                else if (_sample<signalMin)
                    signalMin=_sample;
            }
        }
     _t.reset();
     peakToPeak=signalMax-signalMin;
     _value= (peakToPeak*3.3);
     _value = floor(_value * 100) / 100;
     _sum+=_value;
     _count++;
    }
    _average=_sum/_count;
    _t1.reset();

    return _average;
}

//LED ARRAY FUNCTION:
//Setup array of 4 LEDs based on the current value read by the microphone
//And the average value found during calibration
void MAX4466::led_array(float x ) {

    if (_value<x+0.05) {
            printf("0-0-0-0\n\r");
#if 0            
            _led1=0;
            _led2=0;
            _led3=0;
            _led4=0;
#endif
    }
    if (_value>x+0.05&&_value<0.5+x) {
            printf("1-0-0-0\n\r");
#if 0           
            _led1=1;
            _led2=0;
            _led3=0;
            _led4=0;
#endif            
    }
    if (_value>0.5+x&&_value<1+x) {
            printf("1-1-0-0\n\r");
#if 0           
            _led1=1;
            _led2=1;
            _led3=0;
            _led4=0;
#endif            
    }
    if (_value>1+x&&_value<1.2+x) {
            printf("1-1-1-0\n\r");
#if 0           
            _led1=1;
            _led2=1;
            _led3=1;
            _led4=0;
#endif            
    }
    if (_value>1.2+x&&_value<2.8+x) {
            printf("1-1-1-1\n\r");
#if 0           
            _led1=1;
            _led2=1;
            _led3=1;
            _led4=1;
#endif            
    }
}

//SOUND LEVEL FUNCTIOM:
//Read in current sound level
float MAX4466::sound_level() {

    _t.start();
    float peakToPeak=0, signalMax=0, signalMin=1024;

    while (_t.read_ms()<50) {
        _sample=_pin.read();

        if (_sample<1024) {
            if (_sample>signalMax)
                signalMax=_sample;

            else if (_sample<signalMin)
                signalMin=_sample;
        }
    }
    _t.reset();
    peakToPeak=signalMax-signalMin;
    _value= (peakToPeak*3.3);
    _value = floor(_value * 100) / 100;
    _sum+=_value;
    _count++;

    return _value;
}

double MAX4466::sound() {

    _t.reset();
    _t.start();
    float peakToPeak=0, signalMax=0, signalMin=1024;

    while (_t.read_ms()<50) {
        _sample=_pin.read();

        if (_sample<1024) {
            if (_sample>signalMax)
                signalMax=_sample;

            else if (_sample<signalMin)
                signalMin=_sample;
        }
    }
    _t.reset();
    peakToPeak=signalMax-signalMin;
    _value= (peakToPeak*3.3);
    _value = floor(_value * 100) / 100;
    _sum+=_value;
    _count++;

    // max - min = peak-peak amplitude
    double volts = (peakToPeak * 3.3) * 0.707;  // convert to RMS voltage
    double first = log10(volts/0.00631) * 20;
    double second = first + 94 - 44 - 25;
#if MAX4466_DEBUG
node_printf_to_serial("test data:%f, max:%f, min:%f\n\r", second, signalMax, signalMin); 
#endif    

    return second;
}

//VOLUME INDICATOR FUNCTION:
//To be called in main function
void MAX4466::volume_indicator() {
        led_array(_average);
        sound_level();
}
