#include "RainSensor.h"

RainSensor::RainSensor(int _pin) {
  pin = pin;
}

void RainSensor::initRainSensor() {
  pinMode(pin, INPUT);
}

void RainSensor::clearRainAmount() {
  rainAmount = 0;
  rainCount = 0;
}

void RainSensor::calcRainAmount() {
  //one pulse: 0.33mm rain
  rainAmount += 0.33;
  rainCount++;
}

float RainSensor::getRainAmount(bool clearVars) {
  float temp = rainAmount;
  if (clearVars)
    rainAmount = 0;
    rainCount = 0;
    lastClear=millis();
  return temp;
}

int RainSensor::getRainCount() {
  return rainCount;
}

void RainSensor::setCal(float _rain_mm_pp) {
  rain_mm_pp = _rain_mm_pp;
  }


