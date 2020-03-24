#ifndef WINDSENSOR2_H
#define WINDSENSOR2_H

#include "Arduino.h"
#include <math.h>
#include <QMC5883LCompass.h>

#define SECT_ANGLE 45

#define D_N  0
#define D_NE 1
#define D_E  2
#define D_SE 3
#define D_S  4
#define D_SW 5
#define D_W  6
#define D_NW 7

//windir mode
#define Y_X_p 0
#define Y_X_n 1
#define Y_Z_p 2
#define Y_Z_n 3
#define Z_X_p 4
#define Z_X_n 5

class WindSensor2 {
  private:
    //new section
    QMC5883LCompass compass;
    bool hasMag;
    int magX=-1, magY=-1, magZ=-1;
    byte windir_mode = Y_X_p;
    float windDirAngle = -1;
    float windDirAngleLast = -1;
    int ny=1, nz=0, dx=1, dz=0; //coef. gy-271 orientation
    float offset_deg = 0;
    float corr_coef = 1; //elliptic correction

   
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
    void calculateDir();

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
    void setwdirmode(byte _windir_mode);
    byte getwdirmode();

    //New Setction
    void readCompass();
    int getXmag();
    int getYmag();
    int getZmag();

};

#endif
