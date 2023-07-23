#include <Wire.h>
#include<LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

BlynkTimer timer;

LiquidCrystal_I2C lcd(0x27, 20,4);

char ssid[] = "ML LUTFI";
char pass[] = "okmaju6571";

#define BLYNK_TEMPLATE_ID "TMPL6vU5AcI9-"
#define BLYNK_TEMPLATE_NAME "water level monitoring system"
#define BLYNK_AUTH_TOKEN "6-fqnqBSO3AAOSoa6SgrMT9FqY06Mte6"

#define SENSOR      18
#define TRIG_PIN    34
#define ECHO_PIN    35
#define RELAY_POMPA 26
#define RELAY_VALVE 27
#define BUZZER      13
 
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
long duration_us, distance_cm;

void IRAM_ATTR pulseCounter(){
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  pinMode(SENSOR, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_POMPA, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY_VALVE, OUTPUT);

  pulseCount = 0;
  flowRate = 0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  
  timer.setInterval(1000L, myTimerEvent);
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
}

void loop() {
  Blynk.run();
  timer.run();
  //ULTRASONIK
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
  duration_us = pulseIn(ECHO_PIN, HIGH);
  distance_cm = (duration_us/2)/29.1;
  if (distance_cm <= 25){
    digitalWrite(RELAY_POMPA, LOW);
    digitalWrite(RELAY_VALVE, LOW);
    digitalWrite(BUZZER, LOW);
    lcd.setCursor(0,3);
    lcd.print("    LEVEL 3!    ");}
  else if (distance_cm > 25 && distance_cm <= 70){
    digitalWrite(BUZZER, LOW);
    lcd.setCursor(0,3);
    lcd.print("    LEVEL 2!    ");}
  else if (distance_cm > 70 && distance_cm <= 135){
    digitalWrite(BUZZER, LOW);
    lcd.setCursor(0,3);
    lcd.print("    LEVEL 1!    ");}
  else if(distance_cm > 135){    
    digitalWrite(RELAY_POMPA, HIGH);
    digitalWrite(RELAY_VALVE, HIGH);
    digitalWrite(BUZZER, HIGH);
    lcd.setCursor(0,3);
    lcd.print("    KOSONG!!    ");}

  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  lcd.setCursor(0,1);
  lcd.print("Jarak Air: ");
  lcd.print(distance_cm);
  lcd.println("cm   ");
  delay(500);

  //FLOW SENSOR
  currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    pulse1Sec = pulseCount;
    pulseCount = 0;
    flowRate = ((1000.0 / (millis() - previousMillis)) *pulse1Sec) / calibrationFactor;
    previousMillis = millis();
    flowMilliLitres = (flowRate / 60) *1000;
    totalMilliLitres += flowMilliLitres;
    Serial.print("Flow Rate: ");
    Serial.print(int(flowRate));
    Serial.print("L/min");
    Serial.print("\t");
    lcd.setCursor(0,0);
    lcd.print("Laju Air:");
    lcd.print(int(flowRate));
    lcd.print("L/min ");

    Serial.print("Total Air: ");
    Serial.print("totalMilliLitres");
    Serial.print("mL / ");
    Serial.print(totalMilliLitres / 1000);
    Serial.print("L");
    lcd.setCursor(0,2);
    lcd.print("Total Air: ");
    lcd.print(totalMilliLitres / 1000);
    lcd.print("L   ");
  }
}

void myTimerEvent()
{
 Blynk.virtualWrite(V0,distance_cm);
 Blynk.virtualWrite(V1,totalMilliLitres/1000);
}
