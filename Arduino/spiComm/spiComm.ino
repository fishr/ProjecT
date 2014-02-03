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
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;

void sendByte(byte val){
  digitalWrite(8, LOW);
  SPI.transfer(val);
  digitalWrite(8, HIGH);
}

void sendDMX(byte cmd, byte val){
  sendByte(0xFF);
  sendByte(cmd);
  sendByte(val);
}

void tiltCb( const std_msgs::UInt8& tilt_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  sendDMX(0x0A, tilt_msg.data);
}

void panCb( const std_msgs::UInt8& pan_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  sendDMX(0x09, pan_msg.data);
}

ros::Subscriber<std_msgs::UInt8> subt("tilt", &tiltCb );
ros::Subscriber<std_msgs::UInt8> subp("pan", &panCb );

void setup() {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  //Serial.begin(57600);
  nh.initNode();
  nh.subscribe(subp);
  nh.subscribe(subt);

  // start the SPI library:
  SPI.begin();

}

void loop() {

  nh.spinOnce();
  delay(1);
}
