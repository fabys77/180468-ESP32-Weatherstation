#ifndef WINDSENSOR2_H
#define WINDSENSOR2_H

#include "Arduino.h"
#include <math.h>
#include <QMC5883LCompass.h>


#define D_MARGIN 10

#define D_N  0
#define D_NE 1
#define D_E  2
#define D_SE 3
#define D_S  4
#define D_SW 5
#define D_W  6
#define D_NW 7

class WindSensor2 {
  private:
    //new section
    QMC5883LCompass compass;
    bool hasMag;
    int magX=-1, magY=-1, magZ=-1;
   
    //                  N     NE    E    SE   S    SW    W     NW
    int winDirVal[8] = {3000, 1690, 200, 570, 975, 2368, 3905, 3518};
    
    int windDir = -1;
    int lastDir = -1;
    float windSpeed = 0;

    float windDirAvg = 0;
    long windDirAvgCnt = 0;
    float windSpeedAvg = 0;
    long windSpeedAvgCnt = 0;
    
    unsigned long lastWindSpeedUpdate = 0;
    float deltaWindSpeedUpdate = 0;
    unsigned long windSpeedTimeout = 2500;

    int beaufort[12] = {2, 6, 12, 19, 30, 40, 51, 62, 75, 87, 103, 117}; //km/h (http://www.hko.gov.hk/education/beaufort.htm)
    
    int speedPin;
    int dirPin;

    void addWindDirAvg();
    void addWindSpeedAvg();
    bool timeToRun(unsigned long lastTime, unsigned long interval);

    //calibration data
    uint8_t wind_s_ppr = 2; //WindSpeed sensor: number of pulse per round
    float wind_s_2piR = 0.66; // 2*pi_greco*R, where R is the wind sensor arm lenght in [m]
    

  public:
    WindSensor2(int _speedPin, int _dirPin);
    void initWindSensor();
    void setWindSpeedTimeout(unsigned long nWindSpeedTimeout);
    void updateWindSensor();
    void determineWindDir();
    int getWindDir();
    float getWindDirAvg(bool clearVars = true);
    String getWindDirString();
    void calcWindSpeed();
    float getWindSpeed();
    float getDeltaWindSpeedUpdate();
    float getWindSpeedAvg(bool clearVars = true);
    int getBeaufort();
    String getBeaufortDesc();
    void setCal(uint8_t _wind_s_ppr, float _wind_s_2piR); 
    int getRawADC();

    //New Setction
    void readCompass();
    int getXmag();
    int getYmag();
    int getZmag();

};

#endif
