#define R 1000.0    //known resistor value in the voltage divider

int pin = A5;
float reading, rLight;    //ADC value, photoresistor resistance

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(1000);
  reading = (float)analogRead(pin);   //AD reading decreases as light intensity decreases
  //Rearranged voltage divider equation
  rLight = R * ((1023.0/reading) - 1.0);  //resistance decreases as LI increases
  
  Serial.print("Analog reading: ");
  Serial.println(reading, 1);
  Serial.print("Photresistor resistance: ");
  Serial.println(rLight, 1);
}
