#include "GPS.h"
#include "Message.h"

uint32_t init_speed[5] = {
  9600,19200,38400,57600,115200};

GPS::GPS() //: myserial(10, 11)
{


  lat = 0;            //degree*10 000 000
  lon = 0;            //degree*10 000 000
  time =0;
  ground_speed=0;             // ground speed from gps m/s*100
  altitude=0;                 // gps altitude
  ground_course=0;	          // GPS ground course
  numsats=0;
  gps2dfix=0;
  gps3dfix=0;
  ref_table_now=0;
  _debug_lat = 0;
  for (int i = 0 ;  i < 10; i++) {
    altitude_table[i] = 0;
  }
}


bool GPS::newFrame() {

  char c = SERIAL_L_GPS.read();
  //SERIAL_DEBUG.print(c);
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, gps_frame = NO_FRAME;

  switch (c) {
  case '$': 
    param = 0; 
    offset = 0; 
    parity = 0; 
    break;
  case ',':
  case '*':  
    string[offset] = 0;
    if (param == 0) { //frame identification
      gps_frame = NO_FRAME;  
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') gps_frame = GPGGA_FRAME;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'A') gps_frame = GPGSA_FRAME;
      if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') gps_frame = GPRMC_FRAME;
    }

    switch (gps_frame)
    {
      //************* GPGGA FRAME parsing
    case GPGGA_FRAME: 
      switch (param)
      {
      case 1: 
        time = (atof(string)*1000);      //up to .000 s precision not needed really but the data is there anyway
        break;
        //case 2: i2c_dataset.gps_loc.lat = GPS_coord_to_degrees(string);
      case 2: 
        lat = GPS_coord_to_degrees(string) ;// + _debug_lat;
        break;
        //case 3: if (string[0] == 'S') i2c_dataset.gps_loc.lat = -i2c_dataset.gps_loc.lat;
      case 3: 
        if (string[0] == 'S') 
          lat = - lat;
        break;
        //case 4: i2c_dataset.gps_loc.lon = GPS_coord_to_degrees(string);
      case 4: 
        lon  = GPS_coord_to_degrees(string);
        break;
        //case 5: if (string[0] == 'W') i2c_dataset.gps_loc.lon = -i2c_dataset.gps_loc.lon;
      case 5: 
        if (string[0] == 'W') 
          lon = -lon;
        break;
      case 6: 
        gps2dfix = string[0] ;
        break;
      case 7: 
        numsats = atoi(string);
        break;
      case 9: 
        altitude = atoi(string);
        //altitude++;
        update_table_altitude();
        break;
      }
      break;         
      //************* GPGSA FRAME parsing
    case GPGSA_FRAME:
      #if defined(DEBUG) 
  SERIAL_DEBUG.println("receveice GPGSA_FRAME");
  #endif
      switch (param)
      {
        //case 2: gps3dfix = string[0] == '3';
      case 2: 
        gps3dfix = string[0] ;
        break;
      }
      break;
      //************* GPGSA FRAME parsing
    case GPRMC_FRAME:
      #if defined(DEBUG) 
  SERIAL_DEBUG.println("receveice GPRMC_FRAME");
  #endif
       switch(param)
      {
      case 7: 
        ground_speed = (atof(string)*0.5144444)*10;      //convert to m/s*100
        break; 
      case 8: 
        ground_course = (atof(string)*10);				//Convert to degrees *10 (.1 precision)
        break;
      }

      break;                
    }

    param++; 
    offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
    break;
  case '\r':
  case '\n':  
    if (checksum_param) { //parity checksum
      uint8_t checksum = 16 * ((string[0]>='A') ? string[0] - 'A'+10: string[0] - '0') + ((string[1]>='A') ? string[1] - 'A'+10: string[1]-'0');
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
    break;
  default:
    if (offset < 15) string[offset++] = c;
    if (!checksum_param) parity ^= c;

  }
  if (is_set_home == 0 && numsats >= 5 and ground_speed <= 3 and altitude != 0)  // we need more than 5 sats
  {
    home_lat = lat;
    home_lon = lon;
    home_altitude = altitude;	   //in future height will be set with BMP085
    is_set_home = 1;

  }  
  
#if defined(DEBUG2) 
  SERIAL_DEBUG.print("is_set_home ");
  SERIAL_DEBUG.print(is_set_home);
  SERIAL_DEBUG.print("  numsats ");
  SERIAL_DEBUG.print( numsats);
  
  SERIAL_DEBUG.print(" alt=");
  SERIAL_DEBUG.print(altitude);
  SERIAL_DEBUG.print(" time=");
  SERIAL_DEBUG.print(time);
  SERIAL_DEBUG.print(" ground_speed ");
  SERIAL_DEBUG.print(ground_speed);
  SERIAL_DEBUG.print(" ground course ");
  SERIAL_DEBUG.println(ground_course);
#endif

  return frameOK && (gps_frame == GPGGA_FRAME);
}


void GPS::init() { 

#ifdef USE_SoftwareSerial
  myserial = new SoftwareSerial(SoftwareSerial_in,SoftwareSerial_out);
#endif
  is_set_home = 0;

  SERIAL_L_GPS.begin(GPS_SERIAL_SPEED);  
  delay(1000);
  uint8_t i;
#if defined(INIT_MTK_GPS)                            // MTK GPS setup
  for(uint8_t i=0;i<5;i++){
    delay(100);
    SERIAL_L_GPS.begin(init_speed[i]);
    delay(100);	
    SERIAL_L_GPS.write(MTK_OUTPUT_1HZ);				   // switch UART speed for sending SET BAUDRATE command
#if (GPS_SERIAL_SPEED==9600)
    SERIAL_L_GPS.write("$PMTK251,9600*17\r\n");     // 19200 baud - minimal speed for 5Hz update rate
#endif  
#if (GPS_SERIAL_SPEED==14400)
    SERIAL_L_GPS.write("$PMTK251,14400*29\r\n");     // 19200 baud - minimal speed for 5Hz update rate
#endif  
#if (GPS_SERIAL_SPEED==19200)
    SERIAL_L_GPS.write("$PMTK251,19200*22\r\n");     // 19200 baud - minimal speed for 5Hz update rate
#endif  
#if (GPS_SERIAL_SPEED==38400)
    SERIAL_L_GPS.write("$PMTK251,38400*27\r\n");     // 38400 baud
#endif  
#if (GPS_SERIAL_SPEED==57600)
    SERIAL_L_GPS.write("$PMTK251,57600*2C\r\n");     // 57600 baud
#endif  
#if (GPS_SERIAL_SPEED==115200)
    SERIAL_L_GPS.write("$PMTK251,115200*1F\r\n");    // 115200 baud
#endif  
    delay(300);
  }
  // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
  SERIAL_L_GPS.begin(GPS_SERIAL_SPEED);

  SERIAL_L_GPS.write(MTK_NAVTHRES_OFF);
  delay(100);
  SERIAL_L_GPS.write(SBAS_ON);
  delay(100);
  SERIAL_L_GPS.write(WAAS_ON);
  delay(100);
  SERIAL_L_GPS.write(SBAS_TEST_MODE);
  delay(100);
  SERIAL_L_GPS.write(MTK_OUTPUT_5HZ);           // 5 Hz update rate
  delay(100);


  SERIAL_L_GPS.write(MTK_SET_NMEA_SENTENCES); // only GGA and RMC sentence
#endif     
}


#define DIGIT_TO_VAL(_x)	(_x - '0')
uint32_t GPS::GPS_coord_to_degrees(char* s)
{
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++)
    ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (int i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
};

bool GPS::available()
{
  return SERIAL_L_GPS.available()  ;
}

uint32_t GPS::seconds() {
  return millis() / 1000;
}

void GPS::update_table_altitude()
{
  static uint32_t table_now = 0;
  uint32_t now = seconds();

  if ((now - table_now) >= 1)
  {
    table_now = now ;
    for (int i = 9 ;  i >= 0; i--) {
      altitude_table[i+1] = altitude_table[i];

    }
    altitude_table[0] = altitude ;
  }
}

uint32_t GPS::distance()
//(float lat1, float long1, float lat2, float long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers

  float long1 = (float) lon / 1000000 ;
  float long2 = (float) home_lon / 1000000 ;
  float lat1 = (float) lat / 1000000;
  float lat2 = (float) home_lat / 1000000 ;


  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}






