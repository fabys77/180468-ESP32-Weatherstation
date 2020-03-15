#ifndef RAINSENSOR_H
#define RAINSENSOR_H

#include "Arduino.h"

class RainSensor {
  private:
    int pin;
    
    float rainAmount = 0; //in mm
    int rainCount = 0;
    uint32_t lastClear=0;
    float rain_mm_pp =0.33;
  public:
    RainSensor(int _pin);
    void initRainSensor();
    void clearRainAmount();
    void calcRainAmount();
    float getRainAmount(bool clearVars = true);
    float getRainCurrentAmount( void );
    int getRainCount();
    void setCal(float _rain_mm_pp);
};

#endif
