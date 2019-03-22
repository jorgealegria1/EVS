#include "Arduino.h"
#include <math.h>

float fTemp, cTemp, kTemp;   //Fahrenheit, Celsisus, Kelvin
float inverseTemp;    //result of simplified Steinhart-Hart eqn.
float tResistance;  //thermal resistance
float adcReading;
float roomT = 298.15;  //room temperature in Kelvins
float roomR = 10000;   //thermistor resistance at room temperature
float R = 10000;   //10k resistor used in voltage divider
float B = 3950;   //thermistor coefficient
int i = 0;

#define pin A0   //adjust to whichever analog pin is connected

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);
}

void loop() {
  delay(2000);  //2 second delay

  adcReading = analogRead(pin);
  tResistance = 1023 - adcReading;
  tResistance = (adcReading * R)/tResistance;

  //Resistance to Temperature Conversion
  inverseTemp = log(tResistance/roomR);
  inverseTemp = inverseTemp/B;
  inverseTemp = (1/roomT) + inverseTemp;
  kTemp = 1/inverseTemp;
  cTemp = kTemp - 273.15;       //converts from Kelvin to Celsius
  fTemp = (cTemp * 1.8) + 32.0;   //converts from Celsius to Fahrenheit
  
  //Used to discard innaccurate first analogRead() value
  if (i > 0){
    Serial.print("Resistance: ");
    Serial.print(tResistance);
    Serial.print("\t\tTemperature: ");
    //Serial.print(cTemp);
    Serial.print(fTemp);
    Serial.print("\n");  
  }
  if (i == 0){
    i++;
  }
}
