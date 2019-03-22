#include <Arduino.h>
#define pin A0   //adjust to whichever pin is connected

static float AD, kPa, PSI;

void setup() {
  Serial.begin(9600);
  delay(20);  //20 mS warm up time (specified in the datasheet)
}

void loop() {
  delay(500);
  
  AD = analogRead(pin);
  /*  Datasheet transer function: P = (1/0.0018)*(Vout/Vin - 0.04)
   *  Vout/Vin = AD/1023
   */
  kPa = (1/0.0018) * ((AD/1023) - 0.04);
  //0.145 PSI = 1.0 kPa
  PSI = kPa * 0.145;

  Serial.print("Pressure in kPa: ");
  Serial.print(kPa);
  Serial.print("\tPressure in PSI: ");
  Serial.print(PSI);
  Serial.print("\n");
}
