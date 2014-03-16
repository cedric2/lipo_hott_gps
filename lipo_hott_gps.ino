//  Programme de gestion double alimentation HottV4 by cedric2 (www.modelisme.com)
//  Thanks to
//  Module General Air
//  www.thyzoon.fr
//  MultiWiiCopter by Alexandre Dubus
//  www.multiwii.com
//  2011-2012
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

// V7 First Version in Google code repository
// V8 add GPS



//
#include <SoftwareSerial.h>
#include <EEPROM.h>

#include "Message.h"



#define TRUE 1
#define FALSE 0

#define NO 0
#define YES 1

// Def LED verte carte Arduino mini pro
#define LEDPIN_PINMODE          pinMode (13, OUTPUT);
#define LEDPIN_SWITCH           PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF              PORTB &= ~(1<<5);
#define LEDPIN_ON               PORTB |= (1<<5);


// Gestion jauge (seuil de tension)
//


// Variables



// 
GMessage message;


// Init
void setup() {
  LEDPIN_PINMODE
    LEDPIN_ON
    message.init();

}



// Boucle pricipale
void loop() {
  message.main_loop();
}




