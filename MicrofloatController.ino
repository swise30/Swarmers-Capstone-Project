#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "MS5837.h"
#include <SPI.h>
#include <RH_RF95.h>

//Pins
#define gpsRX 0
#define gpsTX 1
int motorpin = 2;
int dirpin = 3;
int speedpin = 4;


//Setting parts
SoftwareSerial gpsSerial(gpsRX,gpsTX);
TinyGPS gps; 

MS5837 pressSensor;

//varibles
double desDepth = 10; //desired depth in meters
double currDepth;
double errormax;
#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.
float flat, flon, ftime;
unsigned long age;
int year;
byte mon, day_, hr, minutes, sec;

File myFile;

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(GPS_BAUD);

  pinMode(gpsRX, INPUT);
  pinMode(gpsTX, OUTPUT);
  pinMode(motorpin, OUTPUT);
  pinMode(dirpin, OUTPUT);
  pinMode(speedpin, OUTPUT);
 
  pressSensor.setFluidDensity(1029); // kg/m^3 (freshwater 997, 1029 for seawater)
  digitalWrite(motorpin, HIGH);
}

void loop()
{
  currDepth = pressSensor.depth();
  double depth_error = desDepth -currDepth;
  if(depth_error < 0){
    digitalWrite(motorpin, HIGH);
    digitalWrite(dirpin, LOW);//change dir to CW intake
  }else if (depth_error > 0 && depth_error < errormax){
    digitalWrite(motorpin, HIGH);
    digitalWrite(dirpin, HIGH);//change dir to CCW outake
  }else{
    digitalWrite(motorpin, LOW);
  }
  
  delay(1000);
}

//Function
void printTime()
{
  gps.crack_datetime(&year, &mon, &day_, &hr, &minutes, &sec);
  Serial.print(hr);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.println(sec);
}
// Print latitude, longitude, altitude in feet, course, speed, date, time,
void printGPSInfo()
{
  if(gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
    gps.f_get_position(&flat, &flon, &age);
    String lati = String(flat,6), lon = String(flon,6);
  
    Serial.println();
    Serial.print("Lat: "); Serial.print(lati);
    Serial.print(" Long: "); Serial.print(lon);
    Serial.print(" Course: "); Serial.print(gps.course());
    Serial.print(" Speed(kmph): "); Serial.print(gps.f_speed_kmph());
    Serial.print(" Fixed Age: "); Serial.print(age);
    Serial.print(" Time: "); printTime();
    Serial.println();
  }
}

void transmitData(){
  int16_t packetnum = 0;
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();

}
