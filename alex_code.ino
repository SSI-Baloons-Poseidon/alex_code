#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GPS.h>
//We have the following electrical components to control:
//BMP280 -- pressure (altitute) and internal temperature, use I2C
//Thermocouple -- external temperature, use SPI
//Heater -- use digital write
//SD card -- data logging and error logging
//GPS module -- positioning information, use UART
//Rockblock -- communication, use UART
//Hold UART0 for debugging
#define HEATER_PIN 6
#define HEATER_SETPOINT 0
#define THERMOCOUPLE_CS_PIN 5
#define SD_CS_PIN 4
#define BMP_CS_PIN 3 //actual
#define GROUND_PRESSURE 1013.25
#define WRITE_INTERVAL 2000
#define PRINT_TIME 1000
Adafruit_MAX31855 thermocouple(THERMOCOUPLE_CS_PIN);
Adafruit_BMP280 bme(BMP_CS_PIN);
Adafruit_GPS GPS(&Serial1); //pins 19 and 18
#define GPSECHO true
File data_log, status_log;
double internal_temp = 1;
double external_temp = 1;
double pressure = 0;
double altitude = 0;
uint8_t heater_on = 0;
unsigned long start_time, elapsed_time = 0;
unsigned long print_start_time, print_elapsed_time = 0;
unsigned long gps_timer = 0;
void run_heaters();
void init_sensors();
void update_sensors();
void log_data();
void print_data();
void log_status(String message);
void update_gps();
void log_header();
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
void setup() {
  Serial.begin(9600);
  if(!SD.begin(SD_CS_PIN)){
    Serial.println("No valid SD card found!");
  }
  log_header();
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
    useInterrupt(true);
  init_sensors();
}
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}
void loop() {
  update_sensors();
  update_gps();
  run_heaters();
  log_data();
  print_data();
  delay(20);
}
void update_gps(){
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
}
void log_status(String message){
  Serial.println(message);
  status_log = SD.open("status_log.txt", FILE_WRITE);
  if(status_log){
    status_log.println(message);
  }
  else{
    Serial.println("Error writing to status log!");
  }
}
void log_data(){
  elapsed_time = millis() - start_time;
  if(elapsed_time > WRITE_INTERVAL){
    start_time = millis();
    elapsed_time = 0;
    data_log = SD.open("data_log.csv", FILE_WRITE);
    delay(50);
    if(data_log){
      data_log.print(millis());
      data_log.print(",");
      data_log.print(internal_temp);
      data_log.print(",");
      data_log.print(external_temp);
      data_log.print(",");
      data_log.print(pressure);
      data_log.print(",");
      data_log.print(altitude);
      data_log.print(",");
      if(heater_on){
        data_log.print("ON");
      }
      else{
        data_log.print("OFF");
      }
      data_log.print(",");
      data_log.print(GPS.hour, DEC); data_log.print(':');
      data_log.print(GPS.minute, DEC); data_log.print(':');
      data_log.print(GPS.seconds, DEC); data_log.print('.');
      data_log.print(GPS.milliseconds);
      data_log.print(",");
      data_log.print(GPS.day, DEC); data_log.print('/');
      data_log.print(GPS.month, DEC); data_log.print("/20");
      data_log.print(GPS.year, DEC);
      data_log.print(",");
      data_log.print((int)GPS.fix);
      data_log.print(",");
      data_log.print((int)GPS.fixquality); 
      if (GPS.fix) {
        data_log.print(",");
        data_log.print(GPS.latitude, 4); data_log.print(GPS.lat);
        data_log.print(","); 
        data_log.print(GPS.longitude, 4); data_log.print(GPS.lon);
        data_log.print(",");
        data_log.print(GPS.speed);
        data_log.print(",");
        data_log.print(GPS.angle);
        data_log.print(",");
        data_log.print(GPS.altitude);
        data_log.print(","); 
        data_log.print((int)GPS.satellites);
    }
    data_log.println("");
    data_log.close();
    }
    else{
      Serial.println("Error writing to data log!");
    }
  }
}
void log_header(){
  data_log = SD.open("data_log.csv", FILE_WRITE);
  delay(500);
  if(data_log){
    data_log.println("Time,Internal temp,External temp,Pressure,Altitude,Heater Status,GPS Time,GPS Date,GPS Fix,GPS Fix Quality,GPS Latitude,GPS Longitude,GPS Speed,GPS Angle,GPS Altitude,GPS Satellites");
    data_log.close();
  }
}
void print_data(){
  print_elapsed_time = millis() - print_start_time;
  if(print_elapsed_time > PRINT_TIME){
    print_start_time = millis();
    print_elapsed_time = 0;
    Serial.print("Time since power on: ");
    Serial.println(millis());
    Serial.print("Internal temp: ");
    Serial.println(internal_temp);
    Serial.print("External temp: ");
    Serial.println(external_temp);
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.print("Altitude: ");
    Serial.println(altitude);
    Serial.print("Heater status: ");
    if(heater_on){
        Serial.print("ON");
    }
    else{
      Serial.print("OFF");
    }
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    Serial.println("");
  }
}
void update_sensors(){
  internal_temp = bme.readTemperature();
  pressure = bme.readPressure();
  altitude = bme.readAltitude(GROUND_PRESSURE);
  if(!isnan(thermocouple.readCelsius())) {
    external_temp =  thermocouple.readCelsius();
  }
}
void init_sensors(){
  if(!bme.begin()){
    Serial.println("No valid BMP280 sensor found!");
    while(1);
  }
  if(isnan(thermocouple.readCelsius())){
    Serial.println("No valid thermocouple sensor found!");
    while(1);
  }
}
void run_heaters(){
  if(internal_temp < HEATER_SETPOINT){
    digitalWrite(HEATER_PIN, HIGH);
    heater_on = 1;
  }
  else{
    digitalWrite(HEATER_PIN, LOW);
    heater_on = 0;
  }
}
