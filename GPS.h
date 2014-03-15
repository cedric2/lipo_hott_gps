#ifndef GPS_h
#define GPS_h

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <inttypes.h>

#define SERIAL_L_GPS Serial2

//#define USE_SoftwareSerial
#ifdef USE_SoftwareSerial
#define SERIAL_L_GPS (*myserial)
#endif

#define SoftwareSerial_in 10
#define SoftwareSerial_out 11

//


#define GPS_SERIAL_SPEED 14400

#define NO_FRAME    0
#define GPGGA_FRAME 1
#define GPGSA_FRAME 2
#define GPRMC_FRAME 3



#define MTK_SET_BINARY          "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_SET_NMEA            "$PGCMD,16,1,1,1,1,1*6B\r\n"
#define MTK_SET_NMEA_SENTENCES  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
#define MTK_OUTPUT_1HZ          "$PMTK220,1000*1F\r\n"
#define MTK_OUTPUT_4HZ          "$PMTK220,250*29\r\n"
#define MTK_OUTPUT_5HZ          "$PMTK220,200*2C\r\n"
#define MTK_OUTPUT_10HZ         "$PMTK220,100*2F\r\n"
#define MTK_NAVTHRES_OFF        "$PMTK397,0*23\r\n" // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s  
#define SBAS_ON                 "$PMTK313,1*2E\r\n"
#define WAAS_ON                 "$PMTK301,2*2E\r\n"
#define SBAS_TEST_MODE			 "$PMTK319,0*25\r\n"	//Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)


#define INIT_MTK_GPS
#define NMEA



class GPS {
public:
  GPS();
  bool newFrame() ;


  long      lat;            //degree*10 000 000
  long      lon;            //degree*10 000 000
  long      home_lat;            //degree*10 000 000
  long      home_lon;            //degree*10 000 000

  uint32_t	time;
  uint16_t      ground_speed;             // ground speed from gps m/s*100
  int16_t       altitude;                 // gps altitude
  int16_t       home_altitude;                 // gps altitude
  uint16_t	ground_course;	          // GPS ground course
uint8_t    numsats:
  4;
uint8_t    gps2dfix:
  1;
uint8_t    gps3dfix:
  1;
  void init();
  bool available();
 uint16_t altitude_table[11];
 uint32_t distance();

private:
uint32_t _debug_lat;
  uint32_t seconds();
  void update_table_altitude();
 
  uint8_t ref_table_now;
  uint32_t GPS_coord_to_degrees(char* s);
  uint8_t is_set_home;
#ifdef USE_SoftwareSerial
  SoftwareSerial *myserial;
#endif 
  // RX, TX


};

#endif //GPS_h


