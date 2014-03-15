#ifndef MESSAGE_h
#define MESSAGE_h



#include "Arduino.h"
#include "Tension_Lipo.h"


////  Programme de gestion double alimentation HottV4 by cedric2 (www.modelisme.com)
//  Thanks to
//  Module General Air
//  www.thyzoon.fr
//  MultiWiiCopter by Alexandre Dubus
//  www.multiwii.com
//  2011-2012
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

//#define MODULE_GPS_INCLUDE  //if module gps connected




#if defined(MODULE_GPS_INCLUDE)
#include "GPS.h"
#endif


//#define DEBUG
//#define SERIAL_DEBUG Serial  //use Serial if you want to have message on screen


#define LIPO_MAX 420  //              Max Lipo volatage
#define LIPO_ALARM_DEFAULT 360   //Default alarm value
#define LIPO_MIN 350  //Min Lipo volatage

#define TMAX 235      // Temperature max of temp gauge
#define TMIN 10       // Temperature min of temp gauge

// Baudrate UART
#define SERIAL_HOTT Serial      //I use Serial with a UNO, and Serial3 with a mega, for use Serial for debug
#define SERIAL_COM_HOTT_SPEED 		19200



#if not defined(DEBUG) 
#define WITH_ALARME
#endif

#define ALARME_TENSION_SEUIL 0x10
#define ALARME_TENSION_MINI  0x15
#define ALARME_OFF 0x00

#define HOTT_KEY_RIGHT  14
#define HOTT_KEY_DOWN   11
#define HOTT_KEY_UP     13
#define HOTT_KEY_SET    9
#define HOTT_KEY_LEFT   7

#define HOTTV4_TX_DELAY 650



#if defined(DEBUG_INPUT) 
  #define HOTTV4_ELECTRICAL_AIR_MODULE    '1' // Electric Air Module ID
  #define HOTT_BINARY_MODE_REQUEST_ID	'2'
  #define HOTT_TEXT_MODE_REQUEST_ID      '3'
  #define HOTT_GAM_SENSOR_ID    '4'
  #define HOTT_GPS_SENSOR_ID '5'
#else
  //Graupner #33620 Electric Air Module
  #define HOTTV4_ELECTRICAL_AIR_MODULE    0x8E // Electric Air Module ID
  #define HOTT_BINARY_MODE_REQUEST_ID	0x80
  #define HOTT_TEXT_MODE_REQUEST_ID       0x7f
  #define HOTT_GAM_SENSOR_ID    0x8d
  #define HOTT_GAM_SENSOR_TEXT_ID    0xd0
//   #define HOTT_GAM_SENSOR_ID    '5'
      
  #define HOTT_GPS_SENSOR_ID 0x8a // GPS Sensor ID
  #define HOTT_GPS_SENSOR_TEXT_ID 0xA0 // GPS Module ID

  #define HOTT_VARIO_SENSOR_ID 0x89 // Vario Sensor ID
  #define HOTT_VARIO_SENSOR_TEXT_ID 0x90 // Vario Module ID
#endif





#define HOTT_TELEMETRY_GPS_SENSOR_ID    0x8a
//Graupner #33601 Vario Module
#define HOTT_TELEMETRY_VARIO_SENSOR_ID  0x89



#define HOTT_TEXTMODE_MSG_TEXT_LEN 168
struct HOTT_TEXTMODE_MSG {
  byte start_byte;	//#01 constant value 0x7b
  byte esc;			//#02 constant value 0x00 or 0x01 to exit
  byte warning_beeps;	//#03 1=A 2=B ...
  byte text[8][21];	//#04 ASCII text to display to
  // Bit 7 = 1 -> Inverse character display
  // Display 21x8
  byte stop_byte;		//#171 constant value 0x7d
  byte parity;		//#172 Checksum / parity
};

struct HOTT_GPS_MSG{
  uint8_t startByte;               /* Byte 1: 0x7C = Start byte data */
  uint8_t sensorID;                /* Byte 2: 0x8A = GPS Sensor */
  uint8_t warning_beeps;               /* Byte 3: 0…= warning beeps */
  uint8_t sensorTextID;            /* Byte 4: 160 0xA0 Sensor ID Neu! */
  uint8_t alarmInverse1;           /* Byte 5: 01 inverse status */
  uint8_t alarmInverse2;           /* Byte 6: 00 inverse status status 1 = kein GPS Signal */
  uint8_t flightDirection;         /* Byte 7: 119 = Flightdir./dir. 1 = 2°; 0° (North), 9 0° (East), 180° (South), 270° (West) */
  uint16_t GPSSpeed;             /* Byte 8: 8 = /GPS speed low byte 8km/h */
  
  uint8_t LatitudeNS;              /* Byte 10: 000 = N = 48°39’988 #10 north = 0, south = 1*/
  uint16_t LatitudeMin;          /* Byte 11: 231 0xE7 = 0x12E7 = 4839 */
  uint16_t LatitudeSec;          /* Byte 13: 171 220 = 0xDC = 0x03DC =0988 */

 
  uint8_t longitudeEW;            /* Byte 15: 000  = E= 9° 25’9360 east = 0, west = 1*/
  uint16_t longitudeMin;         /* Byte 16: 150 157 = 0x9D = 0x039D = 0925 */
  uint16_t longitudeSec;         /* Byte 18: 056 144 = 0x90 0x2490 = 9360*/
  
  uint16_t distance;             /* Byte 20: 027 123 = /distance low byte 6 = 6 m */
  uint16_t altitude;             /* Byte 22: 243 244 = /Altitude low byte 500 = 0m */
  uint16_t climbrate1s;                       //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
  //#27
  int8_t climbrate3s;                       //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
  
  uint8_t GPSNumSat;               /* Byte 27: GPS.Satelites (number of satelites) (1 byte) */
  uint8_t GPSFixChar;              /* Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
  uint8_t HomeDirection;           /* Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) */
  uint8_t angleXdirection;         /* Byte 30: angle x-direction (1 byte) */
  uint8_t angleYdirection;         /* Byte 31: angle y-direction (1 byte) */
  uint8_t angleZdirection;         /* Byte 32: angle z-direction (1 byte) */
  int8_t gps_time_h;  //#33 UTC time hours
  int8_t gps_time_m;  //#34 UTC time minutes
  int8_t gps_time_s;  //#35 UTC time seconds
  int8_t gps_time_sss;//#36 UTC time milliseconds

  
  int16_t msl_altitude;  //#37 mean sea level altitude


  uint8_t vibration;               /* Byte 39: vibration (1 bytes) */
  uint8_t Ascii4;                  /* Byte 40: 00 ASCII Free Character [4] */
  uint8_t Ascii5;                  /* Byte 41: 00 ASCII Free Character [5] */
  uint8_t GPS_fix;                 /* Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX */
  uint8_t version;                 /* Byte 43: 00 version number */
  uint8_t endByte;                 /* Byte 44: 0x7D Ende byte */
  uint8_t parity;                  /* Byte 45: Parity Byte */
} ;

//Vario

struct HoTTV4VarioModule {
  uint8_t startByte;
  uint8_t sensorID;
  uint8_t warning_beeps; /* Alarm */
  uint8_t sensorTextID;
  uint8_t alarmInverse;

  uint16_t altitude;
  uint16_t maxAltitude;
  uint16_t minAltitude;

  uint16_t m1s;
  uint16_t m3s;
  uint16_t m10s;

  uint8_t text[24];
  uint8_t empty;

  uint8_t version;
  uint8_t endByte;
  uint8_t parity;
} ;


struct HOTT_GAM_MSG {
  byte startByte;                        //#01 start byte constant value 0x7c
  byte sensorID;             //#02 EAM sensort id. constat value 0x8d
  byte warning_beeps;                     //#03 1=A 2=B ... 0x1a=Z  0 = no alarm
  // Q    Min cell voltage sensor 1
  // R    Min Battery 1 voltage sensor 1
  // J    Max Battery 1 voltage sensor 1
  // F    Min temperature sensor 1
  // H    Max temperature sensor 1
  // S    Min Battery 2 voltage sensor 2
  // K    Max Battery 2 voltage sensor 2
  // G    Min temperature sensor 2
  // I    Max temperature sensor 2
  // W    Max current
  // V    Max capacity mAh
  // P    Min main power voltage
  // X    Max main power voltage
  // O    Min altitude
  // Z    Max altitude
  // C    negative difference m/s too high
  // A    negative difference m/3s too high
  // N    positive difference m/s too high
  // L    positive difference m/3s too high
  // T    Minimum RPM
  // Y    Maximum RPM

  byte sensorTextID;             //#04 constant value 0xd0
  byte alarm_invers1;                     //#05 alarm bitmask. Value is displayed inverted
  //Bit#  Alarm field
  // 0    all cell voltage
  // 1    Battery 1
  // 2    Battery 2
  // 3    Temperature 1
  // 4    Temperature 2
  // 5    Fuel
  // 6    mAh
  // 7    Altitude
  byte alarm_invers2;                     //#06 alarm bitmask. Value is displayed inverted
  //Bit#  Alarm Field
  // 0    main power current
  // 1    main power voltage
  // 2    Altitude
  // 3    m/s                            
  // 4    m/3s
  // 5    unknown
  // 6    unknown
  // 7    "ON" sign/text msg active
  byte cell[6];
  uint16_t  Battery1;                           //#13 battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V
  uint16_t  Battery2;                             //#15 battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V
  byte temperature1;                      //#17 temperature 1. offset of 20. a value of 20 = 0°C
  byte temperature2;                      //#18 temperature 2. offset of 20. a value of 20 = 0°C
  byte fuel_procent;                      //#19 Fuel capacity in %. Values 0--100
  // graphical display ranges: 0-25% 50% 75% 100%
  uint16_t fuel_ml;                         //#20 Fuel in ml scale. Full = 65535!

  uint16_t rpm;                                     //#22 RPM in 10 RPM steps. 300 = 3000rpm

    uint16_t altitude;                        //#24 altitude in meters. offset of 500, 500 = 0m
  //#25
  uint16_t climbrate1s;                       //#26 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
  //#27
  int8_t climbrate3s;                       //#28 climb rate in m/3sec. Value of 120 = 0m/3sec
  uint16_t current;                         //#29 current in 0.1A steps
  uint16_t main_voltage;            //#31 Main power voltage using 0.1V steps
  uint16_t batt_cap;                        //#33 used battery capacity in 10mAh steps
  //#34
  uint16_t speed;                           //#35 (air?) speed in km/h(?) we are using ground speed here per default
  //#36
  byte min_cell_volt;                     //#37 minimum cell voltage in 2mV steps. 124 = 2,48V
  byte min_cell_volt_num;         //#38 number of the cell with the lowest voltage
  uint16_t rpm2;                            //#39 RPM in 10 RPM steps. 300 = 3000rpm                             //#40
  byte general_error_number;      //#41 Voice error == 12. TODO: more docu
  byte pressure;                          //#42 Pressure up to 16bar. 0,1bar scale. 20 = 2bar
  byte version;                           //#43 version number TODO: more info?
  byte endByte;                         //#44 stop byte
  byte parity;                            //#45 CRC/Parity (calculated dynamicaly)
};




class GMessage {

private:
  uint8_t checksum (uint8_t *data);
  void _hott_send_telemetry_data();
  void _hott_send_msg(byte *buffer, int len);
  void _hott_send_text_msg();
  char * _hott_invert_all_chars(char *str);
  char * _hott_invert_chars(char *str, int cnt);
  void init_gam_msg();
  void init_gps_msg();
  void send(int lenght);
  void send2();
  void  _hott_send_msg2(byte *buffer, int len) ;
  void _hott_send_text_msg2();
  void _hott_invert_ligne(int ligne) ;
  void setMainVoltage(uint16_t amp);
  void setVoltage1(uint16_t volt);
  void setVoltage2(uint16_t volt);
  void setLipo(uint16_t volt,int lipo);
  void setAmpere(uint16_t amp);
  void setMilliAmpere(uint16_t mA);
  void setAltitudeInMeters(uint16_t alti);
  void setClimbrate(uint16_t sr);
  void setClimbrate_M3(int srmmm);
  void setTemp (int temp, int capteur);
  void setFuelPercent(int pourcent);
  void setFuelMilliliters(uint16_t ml);
  void setRPM(uint16_t rpm);
  void alarme(uint8_t son);
  uint16_t read_eprom(int address);
  void write_eprom(int address,uint16_t val) ;
  
#if defined(MODULE_GPS_INCLUDE)
  GPS gps;
  #endif
  int tmp1 ;


public:
  GMessage();
  void init();
  void main_loop();
};


#endif //GPS_h


