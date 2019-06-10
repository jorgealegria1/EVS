#define batteryPin A4
//Voltage divider resistors
#define R1 9938.0
#define R2 5049.0
//Battery levels
#define minBattery 4.0
#define maxBattery 7.5
#define updatedMaxRange 3.5
float reading, voltage, vin, percent;

// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 22, en = 13, d4 = 12, d5 = 24, d6 = 11, d7 = 26;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(115200);
  lcd.begin(20, 4);
  lcd.setCursor(0, 0);
  delay(500);
}

void loop() {
  reading = analogRead(batteryPin);
  voltage = (reading * 5.0) / 1023.0;
  vin = voltage * ((R1 + R2)/R2);
  //Serial.print(vin);
  //Mapping vin to percentage
  percent = ((vin-4.0)/updatedMaxRange) * 100;
  //  if (percent < 10.0){
  //    Serial.println("Low Battery");
  //  } else {
  //    Serial.println(percent);
  //  }
  lcd.setCursor(0, 0);
  lcd.print("vin: ");
  lcd.print(vin);
  lcd.setCursor(0, 1);
  lcd.print("Battery: ");
  lcd.print(percent);
  delay(500);
}
