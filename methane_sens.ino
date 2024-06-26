#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

const byte MQ4_Pin = A0;
const int R_O = 945;

#define SS_PIN 4
#define RST_PIN 9
#define DI0_PIN 2

#define LEDPIN 1

File myFile;
RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
};

void setup () {
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();

  // SETUP RTC MODULE
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1);
  }

  if (!SD.begin(SS_PIN)) {
    Serial.println("Starting SD card failed!");
    while (1);
  }
  Serial.println("SD card initialized.");
}

void loop () {
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println(getMethanePPM2());
  Serial.println(getMethanePPM());

  DateTime now = rtc.now();

  int yr = now.year();
  int mon = now.month();
  int dt = now.day();

  int hrs = now.hour();
  int mts = now.minute();
  int sec = now.second();

  String currentTime = String(hrs) + ":" + String(mts) + ":" + String(sec);
  Serial.println(currentTime);
  String currentDate = String(yr) + "-" + String(mon) + "-" + String(dt);
  Serial.println(currentDate);
  String currentDay = String(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.println(currentDay);

  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    //myFile.print((float)max(getMethanePPM(), getMethanePPM2()));
    myFile.print((float)getMethanePPM());
    myFile.print(", ");
    myFile.print((float)getMethanePPM2());
    myFile.print(", ");
    myFile.print(currentTime);
    myFile.print(", ");
    myFile.print(currentDate);
    myFile.print(", ");
    myFile.print(currentDay);
    myFile.print(", ");
    myFile.println(LoRa.packetRssi());
    //delay(500);
    myFile.close();
    Serial.println("Data saved to SD card.");
    digitalWrite(LEDPIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println("Error opening data.txt");
  }

  delay(1000);
}

float getMethanePPM(){
  float a0 = analogRead(A0); //get raw reading from sensor
  float v_o = a0 * 5 / 1023; //convert reading to volts
  float R_S = (5-v_o) * 1000 / v_o; //apply formula for getting RS
  float PPM = pow(R_S/R_O, -2.95) * 1000; //apply formula for getting PPM
  float PPM2 = KALMAN(PPM)*2;
  return PPM2; //return PPM value to calling function  
}

float getMethanePPM2(){
  float a0 = analogRead(A1); //get raw reading from sensor
  float v_o = a0 * 5 / 1023; //convert reading to volts
  float R_S = (5-v_o) * 1000 / v_o; //apply formula for getting RS
  float PPM = pow(R_S/R_O, -2.95) * 1000; //apply formula for getting PPM
  float PPM2 = KALMAN(PPM)*2;
  return PPM2; //return PPM value to calling function  
}

float KALMAN (float U){                                              
  static const float R = 0.33; //noise covariance
  static const float H = 1.00; //measurement map
  //at k = 0 we have these initial condidtions
  //i.e before simulations starts
  static float Q = 15; //initial covariance of estimated
  static float P = 0; //initial error covariance
  static float U_hat = 0; //initial estimated state
  static float K = 0; //initial kalman Gain
  //so U comes in, start the KF process
  K = P*H/(H*P*H+R); //Kalman gain
  U_hat = U_hat+ K*(U-H*U_hat);
  //Update error covariance and project it ahead
  P = (1-K*H)*P + Q;
  return U_hat; 
}

//source code for mq4 sensors can be found below
//https://www.utmel.com/components/how-to-use-mq4-gas-sensor?id=821
//source code for rtc can be found below
//https://arduinogetstarted.com/tutorials/arduino-rtc
//I do not not claim ownership of this code #copyright 10-03-2023
