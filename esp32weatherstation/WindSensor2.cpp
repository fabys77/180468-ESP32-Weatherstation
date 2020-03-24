#include "WindSensor2.h"
#include "math.h"

WindSensor2::WindSensor2(int _speedPin, int _dirPin) {
  speedPin = _speedPin;
  dirPin = _dirPin;
}

void WindSensor2::initWindSensor() {
  byte error;
  hasMag = false;
  pinMode(speedPin, INPUT);
  pinMode(dirPin, ANALOG);
  Wire.beginTransmission(byte(13)); //0x0D device
  error = Wire.endTransmission();
  if (error == 0) {
    hasMag=true;
    compass.init();
    Serial.println("QMC5883L Found");
    readCompass();
  }else{
    Serial.println("QMC5883L not Found");
    hasMag=false;
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
  if (hasMag){
    readCompass();
    calculateDir();
    windDir=(int)fmod(fmod((windDirAngle+SECT_ANGLE),360),SECT_ANGLE);
    lastDir=windDir;
    addWindDirAvg();}
  else{
    windDir=-1;
  }
}

int WindSensor2::getWindDir() {
  return (windDir != -1) ? windDir : lastDir; //return the last known value if the current value is -1 (undefined)
}

void WindSensor2::addWindDirAvg() {
  //add the new measurement to the previous measurements and calculate the average value
  windDirAvgCnt++;
  float x = cos(windDirAvg * PI / 180) * (windDirAvgCnt - 1) / windDirAvgCnt + cos(windDirAngle) / windDirAvgCnt;
  float y = sin(windDirAvg * PI / 180) * (windDirAvgCnt - 1) / windDirAvgCnt + sin(windDirAngle) / windDirAvgCnt;
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

void WindSensor2::readCompass() {
  if (hasMag){
    // Read compass values
    compass.read();
    // Return XYZ readings
    magX = compass.getX();
    magY = compass.getY();
    magZ = compass.getZ();
    //  Serial.print("X: ");
    //  Serial.print(x);
    //  Serial.print(" Y: ");
    //  Serial.print(y);
    //  Serial.print(" Z: ");
    //  Serial.print(z);
    //  Serial.println();
  }else{
    magX = -1;
    magY = -1;
    magZ = -1;  
  }
}


int WindSensor2::getXmag(){
    return magX;
}

int WindSensor2::getYmag(){
    return magY;
}

int WindSensor2::getZmag(){
    return magZ;
}

void WindSensor2::calculateDir(){
  double windir_rad;
  windir_rad=atan2((corr_coef*(ny*magY+nz*magZ)),(dx*magX+dz*magZ));
  windDirAngle = (float) fmod(((windir_rad+2*PI)*180/PI),360.0);
}

void WindSensor2::setwdirmode(byte _windir_mode){
  windir_mode=_windir_mode;
  switch (_windir_mode) {
    case Y_X_p:
      ny=1;
      nz=0; 
      dx=1; 
      dz=0;
      break;
   case Y_X_n:
      ny=-1;
      nz=0; 
      dx=1; 
      dz=0;
      break;
   case Y_Z_p:
      ny=1;
      nz=0; 
      dx=0; 
      dz=1;
      break;
   case Y_Z_n:
      ny=-1;
      nz=0; 
      dx=0; 
      dz=1;
      break;
   case Z_X_p:
      ny=0;
      nz=1; 
      dx=1; 
      dz=0;
      break;
   case Z_X_n:
      ny=0;
      nz=-1; 
      dx=1; 
      dz=0;
      break;
   default:      
      ny=1;
      nz=0; 
      dx=1; 
      dz=0;
  }
}

byte WindSensor2::getwdirmode(){
  return windir_mode;

}


