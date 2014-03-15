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

#define LEDPIN_OFF              PORTB &= ~(1<<5);
#define LEDPIN_ON               PORTB |= (1<<5);




#include "Message.h"
#include "Tension_Lipo.h"
#include <EEPROM.h>





// Trame message reponse
// 0x8D : Module General Air


static uint8_t _hott_serial_buffer[173];   //création d'une variable tampon pour stocker les struct

// pointer des structures vers le tampon "_hott_serial_buffer"
struct HOTT_GAM_MSG     *hott_gam_msg = (struct HOTT_GAM_MSG *)&_hott_serial_buffer[0];
struct HOTT_GPS_MSG  *hott_gps_msg = (struct HOTT_GPS_MSG *)&_hott_serial_buffer[0];
struct HOTT_TEXTMODE_MSG	*hott_txt_msg =	(struct HOTT_TEXTMODE_MSG *)&_hott_serial_buffer[0];

static uint16_t alarm_min1 = LIPO_ALARM_DEFAULT  ;
static uint16_t alarm_min2 = LIPO_ALARM_DEFAULT  ;




//adresse en ROM, pour sauvegarde des mins
#define adr_eprom_test 0
#define adr_eprom_alarm_min1 2
#define adr_eprom_alarm_min2 4
//

GMessage::GMessage(){


}
void GMessage::init(){
#if defined(DEBUG) 
  SERIAL_DEBUG.begin (115200);
  SERIAL_DEBUG.println("start");
#endif
  SERIAL_HOTT.begin (SERIAL_COM_HOTT_SPEED);
  int test = read_eprom(adr_eprom_test);
  if (test != 123)
  {
    write_eprom(adr_eprom_test,123); //to initialise very first time
    write_eprom(adr_eprom_alarm_min1,alarm_min1); //to initialise very first time
    write_eprom(adr_eprom_alarm_min2,alarm_min1); //to initialise
  }
  alarm_min1 = read_eprom(adr_eprom_alarm_min1);
  alarm_min2 = read_eprom(adr_eprom_alarm_min2);
#if defined(MODULE_GPS_INCLUDE)
  gps.init();
#endif
  tmp1 = 0 ;
#if defined(DEBUG) 
  SERIAL_DEBUG.println("End init");
#endif
}

uint16_t GMessage::read_eprom(int address){
  return  (uint16_t) EEPROM.read(address) * 256 + EEPROM.read(address+1) ;
}

void GMessage::write_eprom(int address,uint16_t val){
  EEPROM.write(address, val  / 256);
  EEPROM.write(address+1,val % 256 );
}


void GMessage::init_gam_msg(){
  //met tous à Zero, puis on modifie les constantes
  memset(hott_gam_msg, 0, sizeof(struct HOTT_GAM_MSG));   
  hott_gam_msg->startByte = 0x7c;
  hott_gam_msg->sensorID = HOTT_GAM_SENSOR_ID;
  hott_gam_msg->sensorTextID = HOTT_GAM_SENSOR_TEXT_ID;
  hott_gam_msg->endByte = 0x7d;
}


void GMessage::init_gps_msg(){
  //met tous à Zero, puis on modifie les constantes
  memset(hott_gps_msg, 0, sizeof(struct HOTT_GPS_MSG));   
  hott_gps_msg->startByte = 0x7c;
  hott_gps_msg->sensorID = HOTT_GPS_SENSOR_ID;
  hott_gps_msg->sensorTextID = HOTT_GPS_SENSOR_TEXT_ID;
  hott_gps_msg->endByte = 0x7d;
}



// Emission de la trame
void GMessage::send(int lenght){ 
  uint8_t sum = 0;
  delay(5);
  for(int i = 0; i < lenght-1; i++){
    sum = sum + _hott_serial_buffer[i];
    SERIAL_HOTT.write (_hott_serial_buffer[i]);
    delayMicroseconds(HOTTV4_TX_DELAY);
    SERIAL_HOTT.read(); // A cause du rebouclage TX <--> RX Lecture de l'octet émis 
  }  
  //Emision du chechsum
  SERIAL_HOTT.write (sum);
  delayMicroseconds(HOTTV4_TX_DELAY);
  SERIAL_HOTT.read(); // A cause du rebouclage TX <--> RX Lecture de l'octet émis 
}


void GMessage::main_loop(){ 
  static TensionLipo tension;
  
#if defined(MODULE_GPS_INCLUDE)
  while (gps.available()) {
    if (gps.newFrame())
    {
#if defined(DEBUG2) 
      SERIAL_DEBUG.print("read a gps frame");
      SERIAL_DEBUG.println(gps.gps3dfix);
      SERIAL_DEBUG.print("gps2dfix =");
      SERIAL_DEBUG.print(gps.gps2dfix);
      SERIAL_DEBUG.print(" gps3dfix =");
      SERIAL_DEBUG.print(gps.gps3dfix);
      SERIAL_DEBUG.print(" numsats =");
      SERIAL_DEBUG.print(gps.numsats);
      SERIAL_DEBUG.print(" lat =");
      SERIAL_DEBUG.print( gps.lat);
      SERIAL_DEBUG.print(" lon =");
      SERIAL_DEBUG.print( gps.lon);
      SERIAL_DEBUG.print(" lon =");
      SERIAL_DEBUG.print( gps.lon);
      SERIAL_DEBUG.print(" altitude =");
      SERIAL_DEBUG.print( gps.altitude);

      SERIAL_DEBUG.print(" course =");
      SERIAL_DEBUG.print( gps.ground_course);


      SERIAL_DEBUG.print(" speed =");
      SERIAL_DEBUG.println( gps.ground_speed);

#endif
    }
  }
#endif
  if(SERIAL_HOTT.available() >= 2) {
    uint8_t octet1 = SERIAL_HOTT.read();
    switch (octet1) {
    case HOTT_BINARY_MODE_REQUEST_ID:
      { 
        uint8_t  octet2 = SERIAL_HOTT.read();
        switch (octet2) {
#if defined(MODULE_GPS_INCLUDE)
        case HOTT_GPS_SENSOR_ID:
          {
            LEDPIN_OFF
              init_gps_msg();

            if (gps.gps2dfix == 1)
            {

              hott_gps_msg->GPS_fix =  0x33; // Dgps: '0x44' 2D = '0x32' 3D = '0x33' nofix = '0x2d'
              hott_gps_msg->GPSFixChar =  0x33;
              hott_gps_msg->flightDirection = gps.ground_course ;
              hott_gps_msg->GPSSpeed = gps.ground_speed ;
              hott_gps_msg->LatitudeNS = (gps.lat<0) ;
              hott_gps_msg->LatitudeMin = ((gps.lat/10)*6) / 100000 ;
              hott_gps_msg->LatitudeSec =  ((gps.lat/10)*6) % 100000 ;
              hott_gps_msg->longitudeEW = (gps.lon<0) ;
              hott_gps_msg->longitudeMin = ((gps.lon/10)*6) / 100000 ;
              hott_gps_msg->longitudeSec = ((gps.lon/10)*6) % 100000 ;
              hott_gps_msg->altitude = gps.altitude - gps.home_altitude + 500 ;
              hott_gps_msg->distance = gps.distance() ;
              hott_gps_msg->climbrate1s = 30000  + (gps.altitude_table[0] - gps.altitude_table[1])*100 ;
              hott_gps_msg->climbrate3s = 120  + (gps.altitude_table[0] - gps.altitude_table[3]) ;
              //hott_gps_msg->resolution = 
              hott_gps_msg->GPSNumSat = gps.numsats ;

              uint32_t t = gps.time;
              hott_gps_msg->gps_time_h = t / 3600000;
              t -= (hott_gps_msg->gps_time_h * 3600000);

              hott_gps_msg->gps_time_m = t / 60000;
              t -= hott_gps_msg->gps_time_m * 60000;

              hott_gps_msg->gps_time_s = t / 1000;
              hott_gps_msg->gps_time_sss = t - (hott_gps_msg->gps_time_s * 1000);


#if defined(DEBUG2) 
              SERIAL_DEBUG.print("debug fix ");
              SERIAL_DEBUG.print(gps.gps2dfix);
              SERIAL_DEBUG.print(" LatitudeMin =");
              SERIAL_DEBUG.print(hott_gps_msg->LatitudeMin);
              SERIAL_DEBUG.print(" LatitudeSec =");
              SERIAL_DEBUG.print(hott_gps_msg->LatitudeSec);
              SERIAL_DEBUG.print(" Latitudegps =");
              SERIAL_DEBUG.print(gps.lat);
              SERIAL_DEBUG.print(" altitude =");
              SERIAL_DEBUG.print(gps.altitude);
              SERIAL_DEBUG.print(" haltitude =");
              SERIAL_DEBUG.println(gps.home_altitude);
#endif



            }

            else // pas de fix
            {
              hott_gps_msg->GPSFixChar = 0x2d;
              hott_gps_msg->GPS_fix = 0x2d;
              hott_gps_msg->alarmInverse2 = 1;
              hott_gps_msg->altitude = + 500 ;
        
              hott_gps_msg->climbrate1s = 30000  ;
              hott_gps_msg->climbrate3s = 120  ;
            }      
            send(sizeof(struct HOTT_GPS_MSG));
            LEDPIN_ON  
              break;
          }
#endif
        case HOTT_GAM_SENSOR_ID:
          {
            LEDPIN_OFF
              init_gam_msg();
            // Mesure des tensions

          hott_gam_msg->altitude = + 500 ;
        
              hott_gam_msg->climbrate1s = 30000  ;
              hott_gam_msg->climbrate3s = 120  ;
#if defined(MODULE_GPS_INCLUDE)
            hott_gam_msg->altitude = gps.altitude - gps.home_altitude + 500 ;
            hott_gam_msg->climbrate1s = 30000  + (gps.altitude_table[0] - gps.altitude_table[1])*100 ;
            hott_gam_msg->climbrate3s = 120  + (gps.altitude_table[0] - gps.altitude_table[3]) ;
#endif

            uint16_t lipo1 = tension.Tension_Lipo2();  // tension pack en 1/100 (ex 840 = 4.2v)
            uint16_t lipo2 = tension.Tension_Lipo1();  // tension pack en 1/100

            // Configure lipo 1 et 2 : (div by 2, because cannot go up 5v)
            setLipo (lipo1/2,1);
            setLipo (lipo2/2,2);


            setVoltage1(lipo1);    // Affichage sur Batt 1
            setVoltage2(lipo2);    // Affichage sur Batt 2
            uint16_t lipo_mini = min(lipo1,lipo2);
            setMainVoltage(lipo_mini); // Affichage tension minimal sur principal

            if (lipo1/2  <= LIPO_MIN)     //pour eviter d'avoir des valeurs en dessous de 100%
              lipo1 = LIPO_MIN *2 ;
            if (lipo2/2 <= LIPO_MIN)
              lipo2 = LIPO_MIN *2 ;
            lipo_mini = min(lipo1,lipo2);


            int pourcent1 = round(((lipo1/2)-LIPO_MIN)*100/(LIPO_MAX-LIPO_MIN)); //% de capacité
            int pourcent2 = round(((lipo2/2)-LIPO_MIN)*100/(LIPO_MAX-LIPO_MIN));

            setFuelPercent (round(((lipo_mini/2)-LIPO_MIN)*100/(LIPO_MAX-LIPO_MIN)));  //% de la capacité sur fuel

            setTemp(pourcent1*(TMAX-TMIN)/100+TMIN,1); //affiche de la capacité sur la barre température
            setTemp(pourcent2*(TMAX-TMIN)/100+TMIN,2);
            send(sizeof(struct HOTT_GAM_MSG));
            LEDPIN_ON  

              break;
          }
        }

        break;
      }

    case HOTT_TEXT_MODE_REQUEST_ID:
      {
        LEDPIN_OFF
          uint8_t  octet2 = SERIAL_HOTT.read();

        byte id_sensor = (octet2 >> 4);
        byte id_key = octet2 & 0x0f;
        static byte ligne_select = 3 ;
        static int8_t ligne_edit = -1 ;
        hott_txt_msg->start_byte = 0x7b;
        hott_txt_msg->esc = 0;       
        uint16_t lipo1 = tension.Tension_Lipo2();  
        uint16_t lipo2 = tension.Tension_Lipo1();
#if defined(WITH_ALARME)
        if (lipo1/2 <  alarm_min1 || lipo2/2 <  alarm_min2)
          hott_txt_msg->warning_beeps =  ALARME_TENSION_SEUIL    ;
        else
          hott_txt_msg->warning_beeps = 0;
#endif
        memset((char *)&hott_txt_msg->text, 0x20, HOTT_TEXTMODE_MSG_TEXT_LEN);
        hott_txt_msg->stop_byte = 0x7d;

        if (id_key == HOTT_KEY_LEFT)
        {   
          hott_txt_msg->esc = 0x01;
        }
        else
        {
          if (id_sensor == (HOTT_GAM_SENSOR_ID & 0x0f)) 
          {
            if (id_key == HOTT_KEY_UP && ligne_edit == -1)
              ligne_select = min(5,ligne_select+1);
            else if (id_key == HOTT_KEY_DOWN && ligne_edit == -1)
              ligne_select = max(3,ligne_select-1);
            else if (id_key == HOTT_KEY_SET && ligne_edit == -1)
              ligne_edit =  ligne_select ;
            else if (id_key == HOTT_KEY_SET && ligne_edit == 3)
            {
              ligne_edit = -1 ;
              write_eprom(adr_eprom_alarm_min1,alarm_min1);
            }
            else if (id_key == HOTT_KEY_SET && ligne_edit == 4)
            {
              ligne_edit = -1 ;
              write_eprom(adr_eprom_alarm_min2,alarm_min2);
            }        
            else if (id_key == HOTT_KEY_UP && ligne_select == 3 )
              alarm_min1+=10;
            else if (id_key == HOTT_KEY_DOWN && ligne_select == 3 )
              alarm_min1-=10;
            else if (id_key == HOTT_KEY_UP && ligne_select == 4 )
              alarm_min2+=10;
            else if (id_key == HOTT_KEY_DOWN && ligne_select == 4 )
              alarm_min2-=10;

            snprintf((char *)&hott_txt_msg->text[0],21,"Module Double Alim <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Bat1: %d.%02d",lipo1/100,lipo1%100);
            snprintf((char *)&hott_txt_msg->text[2],21,"Bat2: %d.%02d",lipo2/100,lipo2%100);
            snprintf((char *)&hott_txt_msg->text[3],21," Min1: %d.%02d (%d.%02d)",
            alarm_min1/100,alarm_min1%100,alarm_min1/50,(alarm_min1*2)%100);
            snprintf((char *)&hott_txt_msg->text[4],21," Min2: %d.%02d (%d.%02d)",
            alarm_min2/100,alarm_min2%100,alarm_min2/50,(alarm_min2*2)%100);
            //snprintf((char *)&hott_txt_msg->text[5],21," test1 '%d' ",octet3);
            //snprintf((char *)&hott_txt_msg->text[6],21,"s'%d'  e'%d'",ligne_select,ligne_edit);
            //snprintf((char *)&hott_txt_msg->text[7],21,"k '%d'",id_key);
            hott_txt_msg->text[ligne_select][0] = '>';
            _hott_invert_ligne(ligne_edit);
          }
          else if(id_sensor == (HOTT_GPS_SENSOR_ID & 0x0f)) {
            snprintf((char *)&hott_txt_msg->text[0],21,"GPS sensor module  <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }

          else if(id_sensor == (HOTTV4_ELECTRICAL_AIR_MODULE & 0x0f)) {
            snprintf((char *)&hott_txt_msg->text[0],21,"EAM sensor module  <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
          else if(id_sensor == (HOTT_TELEMETRY_VARIO_SENSOR_ID & 0x0f)) {         
            snprintf((char *)&hott_txt_msg->text[0],21,"VARIO sensor module  <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
          }
          else {
            snprintf((char *)&hott_txt_msg->text[0],21,"Unnknow sensor module  <");
            snprintf((char *)&hott_txt_msg->text[1],21,"Nothing here");
            snprintf((char *)&hott_txt_msg->text[1],21,"sensor %d", id_sensor);
          }
        }
        _hott_send_text_msg();
        LEDPIN_ON
          break;

      }
    }	
  }


}


//////////////////////////////////////////////////
// Tension principale (resolution 0.1V)
void GMessage::setMainVoltage(uint16_t tension){
  //hott_gam_msg->current = 10;
  hott_gam_msg->main_voltage = tension / 10;
  //hott_gam_msg->batt_cap = 1000;

}

// Batterie 1 (resolution 0.1V)
void GMessage::setVoltage1(uint16_t volt){
  hott_gam_msg->Battery1 =  volt  /  10  ; 
  if (volt/2 <  alarm_min1)
  {
    hott_gam_msg->alarm_invers1 |= 1<<1;     //affichage
#if defined(WITH_ALARME)
    alarme (ALARME_TENSION_SEUIL);
#endif
  }
  else
    hott_gam_msg->alarm_invers1 &= ~(1<<1);  
}

// Batterie 2 (resolution 0.1V)
void GMessage::setVoltage2(uint16_t volt){
  hott_gam_msg->Battery2 =  volt / 10 ; 
  if (volt/2 <  alarm_min2)
  {
    hott_gam_msg->alarm_invers1 |= 1<<2;
#if defined(WITH_ALARME)
    alarme (ALARME_TENSION_SEUIL);
#endif
  }
  else
    hott_gam_msg->alarm_invers1 &= ~(1<<2);  
}

// Element Lipo n (resolution 0.2V)
void GMessage::setLipo(uint16_t volt, int lipo){

  if (lipo >= 1 and lipo <= 6)
  {
    lipo--;
    hott_gam_msg->cell[lipo] =  (uint16_t) volt / 2 ; 

    if (volt/2 <= hott_gam_msg->min_cell_volt || hott_gam_msg->min_cell_volt ==0)
    {
      hott_gam_msg->min_cell_volt = (uint16_t) volt/2 ;
      hott_gam_msg->min_cell_volt_num = lipo;
    }
  }
}


// Altitude relative en metres : -500 / +9999 [m]
void GMessage::setAltitudeInMeters(uint16_t alti){
  hott_gam_msg->altitude = alti + 500;
}

// Temperature : -20 a +235° C 
// 
void GMessage::setTemp(int temp, int capteur){
  if(temp < -20)
    temp = -20;
  else if(temp > 234)
    temp = 235;
  if (capteur == 1)
    hott_gam_msg->temperature1 = temp + 20;
  else if (capteur == 2)
    hott_gam_msg->temperature2 = temp + 20;      
}

// Niveau reservoir : 0%, 25%, 50%, 75%, 100%  
// 
void GMessage::setFuelPercent(int pourcent){
  //pourcent-=12;
  if(pourcent > 100)
    pourcent = 100;
  else if(pourcent < 0)
    pourcent = 0;
  hott_gam_msg->fuel_procent = pourcent ;
}

// niveau reservoir en ml : 0 a 65535 ml
// 
void GMessage::setFuelMilliliters(uint16_t ml){
  hott_gam_msg->fuel_ml = ml ;
}

// Rotation : 0 a 655350 tr/min
// 

void GMessage::setRPM(uint16_t rpm){
  hott_gam_msg->rpm = rpm ;
} 

//
// Alarme sonore sur radio
//
void GMessage::alarme (uint8_t son){
  hott_gam_msg->warning_beeps = son;
}



void GMessage::_hott_send_text_msg() {
  for(byte *_hott_msg_ptr = hott_txt_msg->text[0]; _hott_msg_ptr < &hott_txt_msg->stop_byte ; _hott_msg_ptr++){
    if (*_hott_msg_ptr == 0)
      *_hott_msg_ptr = 0x20;
  }  
  //_hott_send_msg(_hott_serial_buffer, sizeof(struct HOTT_TEXTMODE_MSG));
  send(sizeof(struct HOTT_TEXTMODE_MSG));
}


char * GMessage::_hott_invert_all_chars(char *str) {
  return _hott_invert_chars(str, 0);
}


char * GMessage::_hott_invert_chars(char *str, int cnt) {
  if(str == 0) return str;
  int len = strlen(str);
  if((len < cnt)  && cnt > 0) len = cnt;
  for(int i=0; i< len; i++) {
    str[i] = (byte)(0x80 + (byte)str[i]);
  }
  return str;
}


void GMessage::_hott_invert_ligne(int ligne) {
  if (ligne>= 0 && ligne<= 7)
    for(int i=0; i< 21; i++) {
      if (hott_txt_msg->text[ligne][i] == 0)   //inversion du caratère null (fin de string)
        hott_txt_msg->text[ligne][i] = (byte)(0x80 + 0x20);
      else
        hott_txt_msg->text[ligne][i] = (byte)(0x80 + (byte)hott_txt_msg->text[ligne][i]);
    }
}












