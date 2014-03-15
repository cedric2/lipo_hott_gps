//  Programme de gestion double alimentation HottV4 by cedric2 (www.modelisme.com)
//  Mesure de tension
//
//
// Auteur version original : www.thyzoon.fr 
//
// 2011-2012
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

#include "Tension_Lipo.h"

TensionLipo::TensionLipo() {
  analogReference(DEFAULT); // reference alim. 5V
  //digitalWrite(A0, LOW); 
  // pinMode(A0, INPUT);
}




uint16_t TensionLipo::Tension_Lipo1(){
  static uint16_t MV1 = 800; //4.20;
  static uint16_t MV2 = 800;
  uint16_t MV3;
  uint16_t val;
  MV3 =  mesure(PIN_BAT1)* COEF_BAT1;
  val = (MV1 + MV2 + MV3) / 3; // filtrage (moyenne de 3 echantillons)
  MV1 = MV2; // décalage
  MV2 = MV3;

  return val;
}

uint16_t TensionLipo::Tension_Lipo2(){
  static uint16_t MV1 = 800; //4.20;
  static uint16_t MV2 = 800;
  uint16_t MV3;
  uint16_t val;
  MV3 =  mesure(PIN_BAT2)* COEF_BAT2;
  val = (MV1 + MV2 + MV3) / 3; // filtrage (moyenne de 3 echantillons)
  MV1 = MV2; // décalage
  MV2 = MV3;
  return val;
}



// Mesure tension (CAN)
uint16_t TensionLipo::mesure(uint8_t pin){
  uint16_t val = analogRead(pin); 
  return val;
}



