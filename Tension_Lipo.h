#ifndef Tension_Lipo_h
#define Tension_Lipo_h
//  Programme de gestion double alimentation HottV4 by cedric2 (www.modelisme.com)
//
//  Mesure de tension
//
//
// Auteur initial : www.thyzoon.fr 
//
// 2011-2012
//


#include "Arduino.h"


// Calcul des coefficeints de mesures en fonction des resistances 
// COEF = ((R1 + R2)/R2) * (VREF/1024) 
// VREF = 5V (refrence interne)

#define COEF_BAT1 0.97656
#define COEF_BAT2 0.97656  

//#define PIN_BAT1 6
//#define PIN_BAT2 7

#define PIN_BAT1 3
#define PIN_BAT2 2

class TensionLipo{
public:
  TensionLipo();
  uint16_t Tension_all (uint16_t i);
  uint16_t Tension_Lipo1 ();
  uint16_t Tension_Lipo2 ();
  uint16_t readVcc ();
private:
  uint16_t mesure(uint8_t pin);
};

#endif Tension_Lipo_h
