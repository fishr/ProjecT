/*
  DMX_Master.ino - Example code for using the Conceptinetics DMX library
  Copyright (c) 2013 W.A. van der Meeren <danny@illogic.nl>.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
#define SS_PIN    10

#include <Conceptinetics.h>


//
// CTC-DRA-13-1 ISOLATED DMX-RDM SHIELD JUMPER INSTRUCTIONS
//
// If you are using the above mentioned shield you should 
// place the RXEN jumper towards pin number 2, this allows the
// master controller to put to iso shield into transmit 
// (DMX Master) mode 
//
//
// The !EN Jumper should be either placed in the G (GROUND) 
// position to enable the shield circuitry 
//   OR
// if one of the pins is selected the selected pin should be
// set to OUTPUT mode and set to LOGIC LOW in order for the 
// shield to work
//


//
// The master will control 100 Channels (1-100)
// 
// depending on the ammount of memory you have free you can choose
// to enlarge or schrink the ammount of channels (minimum is 1)
//
#define DMX_MASTER_CHANNELS   12 

//
// Pin number to change read or write mode on the shield
//
#define RXEN_PIN                2


// Configure a DMX master controller, the master controller
// will use the RXEN_PIN to control its write operation 
// on the bus
DMX_Master        dmx_master ( DMX_MASTER_CHANNELS, RXEN_PIN );
boolean resetFrame=false;
boolean commandFrame=false;
int command=0;

// the setup routine runs once when you press reset:
void setup() {      
  
  
pinMode(SCK_PIN, INPUT);
pinMode(MOSI_PIN, INPUT);
pinMode(SS_PIN, INPUT);
  
  // Enable DMX master interface and start transmitting
  dmx_master.enable ();  
    // Set channel 1 - 50 @ 50%
    //dmx_master.setChannelValue ( 7, 230 ); 
    //dmx_master.setChannelValue ( 1, 50 );
    dmx_master.setChannelValue ( 12, 127 ); 
    dmx_master.setChannelValue ( 11, 127 );
    dmx_master.setChannelValue ( 9, 127 ); 

    dmx_master.setChannelValue ( 10, 127 ); 
 
  SlaveInit();
  //lastSent = millis();
  //digitalWrite(8, LOW);
 

    //dmx_master.setChannelValue ( 6, 200 ); 
//dmx_master.setChannelValue ( 7, 0 );    
//delay(500);
  //dmx_master.setChannelValue ( 1, 20 ); 

}

// the loop routine runs over and over again forever:
void loop() 
{
  byte incomingByte = ReadByte();
  if (incomingByte==255){
    resetFrame=true;
  }else if (((incomingByte==9)||(incomingByte==10))&&resetFrame){
    resetFrame=false;
    commandFrame=true;
    command=incomingByte;
  }else if (commandFrame&&(!resetFrame)){
    commandFrame=false;
    dmx_master.setChannelValue ( command, incomingByte);  
  }
}



void SlaveInit(void) {
  // Set MISO output, all others input
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  pinMode(SS_PIN, INPUT);

  // Enable SPI
  SPCR = B00000000;
  SPCR = (1<<SPE);
}

byte ReadByte(void) {
  while(!(SPSR & (1<<SPIF))) ;
  return SPDR;
}
