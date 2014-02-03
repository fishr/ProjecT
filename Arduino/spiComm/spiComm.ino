/*
 SCP1000 Barometric Pressure Sensor Display
 
 Shows the output of a Barometric Pressure Sensor on a
 Uses the SPI library. For details on the sensor, see:
 http://www.sparkfun.com/commerce/product_info.php?products_id=8161
 http://www.vti.fi/en/support/obsolete_products/pressure_sensors/
 
 This sketch adapted from Nathan Seidle's SCP1000 example for PIC:
 http://www.sparkfun.com/datasheets/Sensors/SCP1000-Testing.zip
 
 Circuit:
 SCP1000 sensor attached to pins 6, 7, 10 - 13:
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13
 
 created 31 July 2010
 modified 14 August 2010
 by Tom Igoe
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>

void setup() {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  Serial.begin(57600);

  // start the SPI library:
  SPI.begin();

  delay(100);
}

void loop() {

  if(Serial.available()){
    byte incomingByte = Serial.read();
    Serial.print("Sending:  ");
    Serial.println(incomingByte);
    digitalWrite(8, LOW);
    SPI.transfer(incomingByte);
    digitalWrite(8, HIGH); 
  }
}
