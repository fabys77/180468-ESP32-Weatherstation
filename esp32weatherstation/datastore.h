#ifndef DATASTORE_H_
 #define DATASTORE_H_
 
typedef struct {
  char user[128];
  char pass[256];
} web_credentials_t;
/* 384 byte */

typedef struct {
  bool enable;
  char mqttservername[129];
  uint16_t mqttserverport;
  char mqttusername[129];
  char mqttpassword[129];
  char mqtttopic[501];
  char mqtthostname[65];
  uint16_t mqtttxintervall;
}mqttsettings_t; /*956 byte */

typedef struct {
  uint8_t wind_s_ppr; //WindSpeed sensor: number of pulse per round
  float wind_s_2piR; // 2*pi_greco*R, where R is the wind sensor arm lenght in [m]
  float rain_mm_pp; //Amount of rain [mm] needs for a rain gauge pulse 
  uint16_t TVOC_base; 
  uint16_t eCO2_base;
  byte windir_mode;
  float magCoef;
  int Xoffset;
  int Yoffset;
  int Zoffset;
  float northOffset;
}calsettings_t;


/**************************************************************************************************
 *    Function      : datastoresetup
 *    Description   : Gets the EEPROM Emulation set up
 *    Input         : none 
 *    Output        : none
 *    Remarks       : We use 4096 byte for EEPROM 
 **************************************************************************************************/
void datastoresetup();

/**************************************************************************************************
 *    Function      : write_credentials
 *    Description   : writes the wifi credentials
 *    Input         : credentials_t
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void write_webcredentials(web_credentials_t c);


/**************************************************************************************************
 *    Function      : read_credentials
 *    Description   : reads the wifi credentials
 *    Input         : none
 *    Output        : credentials_t
 *    Remarks       : none
 **************************************************************************************************/
web_credentials_t read_webcredentials( void );

/**************************************************************************************************
 *    Function      : eepwrite_notes
 *    Description   : writes the user notes 
 *    Input         : uint8_t* data, uint32_t size
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void eepwrite_notes(uint8_t* data, uint32_t size);

/**************************************************************************************************
 *    Function      : eepread_notes
 *    Description   : reads the user notes 
 *    Input         : uint8_t* data, uint32_t size
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void eepread_notes(uint8_t* data, uint32_t size);

/**************************************************************************************************
 *    Function      : eepwrite_mqttsettings
 *    Description   : write the mqtt settings
 *    Input         : mqttsettings_t data
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void eepwrite_mqttsettings(mqttsettings_t data);

/**************************************************************************************************
 *    Function      : eepwrite_mqttsettings
 *    Description   : write the mqtt settings
 *    Input         : mqttsettings_t data
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
mqttsettings_t eepread_mqttsettings( void );

/**************************************************************************************************
 *    Function      : erase_eeprom
 *    Description   : writes the whole EEPROM with 0xFF  
 *    Input         : none
 *    Output        : none
 *    Remarks       : This will invalidate all user data 
 **************************************************************************************************/
void erase_eeprom( void );


/**************************************************************************************************
 *    Function      : write_calsettings
 *    Description   : writes calibration stucture
 *    Input         : calsettings_t
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void write_calsettings(calsettings_t c);

/**************************************************************************************************
 *    Function      : read_calsettings
 *    Description   : reads the wifi credentials
 *    Input         : none
 *    Output        : calsettings_t
 *    Remarks       : none
 **************************************************************************************************/
calsettings_t read_calsettings( void );


#endif
