#include "WindSensor2.h"

WindSensor2::WindSensor2(int _speedPin, int _dirPin, int32_t _sensorID) {
  speedPin = _speedPin;
  dirPin = _dirPin;
  sensorID = _sensorID;
}

void WindSensor2::initWindSensor() {
  pinMode(speedPin, INPUT);
  pinMode(dirPin, ANALOG);
  mag = Adafruit_HMC5883_Unified(sensorID);
  if(!mag.begin()){
    Serial.println("No HMC5883 detected");
    hasMag=false;
    }
  else{
    displaySensorDetails();
    hasMag=true;
    }
}

void WindSensor2::setWindSpeedTimeout(unsigned long nWindSpeedTimeout) {
  windSpeedTimeout = nWindSpeedTimeout;
}

void WindSensor2::updateWindSensor() {
  if (timeToRun(lastWindSpeedUpdate, windSpeedTimeout)) {
    windSpeed = 0;
  }
  //add to average value
  addWindSpeedAvg();
}

void WindSensor2::determineWindDir() {
  //TODO
  int dir = analogRead(dirPin);
  for (int i = 0; i < 8; i++) {
    if ((dir > (winDirVal[i] - D_MARGIN)) && (dir < (winDirVal[i] + D_MARGIN))) {
      windDir = i;
      lastDir = i;
      
      //add to average value
      addWindDirAvg();
      return;
    }
  }
  windDir = -1;

  addWindDirAvg();
}

int WindSensor2::getWindDir() {
  return (windDir != -1) ? windDir : lastDir; //return the last known value if the current value is -1 (undefined)
}

void WindSensor2::addWindDirAvg() {
  //TODO
  //add the new measurement to the previous measurements and calculate the average value
  float windDirAngle = (float)getWindDir() * PI / 4; //calculate wind direction to angle
  windDirAvgCnt++;
  float x = cos(windDirAvg * PI / 180) * (windDirAvgCnt - 1) / windDirAvgCnt + cos((float)windDirAngle) / windDirAvgCnt;
  float y = sin(windDirAvg * PI / 180) * (windDirAvgCnt - 1) / windDirAvgCnt + sin((float)windDirAngle) / windDirAvgCnt;
  windDirAvg = atan2(y, x) * 180 / PI;
}

float WindSensor2::getWindDirAvg(bool clearVars) {
  float temp = windDirAvg + (windDirAvg < 0 ? 360 : 0);
  if (clearVars) {
    windDirAvgCnt = 0;
    windDirAvg = 0;
  }
  return temp;
}

String WindSensor2::getWindDirString() {
  switch (getWindDir()) {
    case D_N:
      return "North";
    case D_NE:
      return "North east";
    case D_E:
      return "East";
    case D_SE:
      return "South east";
    case D_S:
      return "South";
    case D_SW:
      return "South west";
    case D_W:
      return "West";
    case D_NW:
      return "North west";
    default:
      return "Undefined";
  }
}

void WindSensor2::setCal(uint8_t _wind_s_ppr, float _wind_s_2piR){
  wind_s_ppr = _wind_s_ppr;
  wind_s_2piR = _wind_s_2piR;
  }


void WindSensor2::calcWindSpeed() {
  //Serial.println("updating windspeed");
  unsigned long currMillis = millis();
  deltaWindSpeedUpdate = (currMillis - lastWindSpeedUpdate);
  float diff = deltaWindSpeedUpdate * wind_s_ppr; //sensor pulses twice per rotation
  lastWindSpeedUpdate = currMillis;
  if (diff > 10) { //diff > 0.01 s -> 100 hz -> 34 m/s -> 122.4 km/h
    //Serial.println("diff: " + String(diff) + "ms");
    diff /= 1000;
    //Serial.println("diff: " + String(diff) + "s");
    float hz = 1.0/diff;
    //Serial.println("hz: " + String(hz));
    windSpeed = hz * wind_s_2piR; //2.4km/h for 1 rot/s -> 2.4/3.6=0.66m/s
    //Serial.println("windSpeed: " + String(windSpeed));
  }
}

float WindSensor2::getWindSpeedAvg(bool clearVars) {
  float temp = windSpeedAvg;
  if (clearVars) {
    windSpeedAvgCnt = 0;
    windSpeedAvg = 0;
  }
  return temp;
}

float WindSensor2::getDeltaWindSpeedUpdate() {
  return deltaWindSpeedUpdate;
}




float WindSensor2::getWindSpeed() {
  return windSpeed;
}

void WindSensor2::addWindSpeedAvg() {
  windSpeedAvgCnt++;
  windSpeedAvg = windSpeedAvg * (windSpeedAvgCnt - 1) / windSpeedAvgCnt + windSpeed / windSpeedAvgCnt;
}

int WindSensor2::getBeaufort() {
  float kmh = windSpeed * 3.6;
  for (int i = 0; i < 12; i++) {
    if (kmh < beaufort[i])
      return i;
  }
  return 12;
}

String WindSensor2::getBeaufortDesc() {
  switch(getBeaufort()) {
    case 0:
      return "Calm";
    case 1:
    case 2:
      return "Light";
    case 3:
    case 4:
      return "Moderate";
    case 5:
      return "Fresh";
    case 6:
    case 7:
      return "Strong";
    case 8:
    case 9:
      return "Gale";
    case 10:
    case 11:
      return "Storm";
    case 12:
      return "Hurricane";
    default:
      return "";
  }
}

int WindSensor2::getRawADC() {
  int dir = analogRead(dirPin);
  return dir;
}

void WindSensor2::displaySensorDetails()
{
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
}

bool WindSensor2::timeToRun(unsigned long lastTime, unsigned long interval){
  //This function is safety when timer wrap
  unsigned long futureEvent = lastTime + interval; 
  if (futureEvent < interval) {
    //timer will wrap
    if ((futureEvent < millis()) && (millis() < (lastTime + 2*interval)))
        return true;
      else
        return false;
  }
  else {
      if (futureEvent < millis())
        return true;
      else
        return false;
  }
}

void WindSensor2::readMagneticSensor(){
  if (hasMag){
    mag.getEvent(&event);
  }
}

int16_t WindSensor2::getXmag(){
  if (hasMag)
    return event.magnetic.x;
  else
    return -1;
}

int16_t WindSensor2::getYmag(){
  if (hasMag)
    return event.magnetic.y;
  else
    return -1;
}

int16_t WindSensor2::getZmag(){
  if (hasMag)
    return event.magnetic.z;
  else
    return -1;
}










