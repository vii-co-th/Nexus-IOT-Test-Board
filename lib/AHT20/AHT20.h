#ifndef __AHT20_H__
#define __AHT20_H__

#include <Arduino.h>
#include <Wire.h>

#define SDA 21
#define SCL 22
#define SDA2 17
#define SCL2 16

class AHT20{
    
private:

    bool startSensor();
    bool startSensor1();
public:

    void begin();
    bool getSensor(float *h, float *t);
    bool getTemperature(float *t);
    bool getHumidity(float *h);
    void begin1();
    bool getSensor1(float *h, float *t);
    bool getTemperature1(float *t);
    bool getHumidity1(float *h);
};

#endif