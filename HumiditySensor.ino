#define Vin 5.0
#define voltDrop 0.0062    //as temp increases, Vout drops by this amount
#define maxV 3.9
#define minV 0.7
int HPIN = A1;
int TPIN = A2;
float reading, Vout, H, RH, maxAdjust;

float fTemp, cTemp, kTemp;   //Fahrenheit, Celsisus, Kelvin
float inverseTemp;    //result of simplified Steinhart-Hart eqn.
float tResistance;  //thermal resistance
float adcReading;
float roomT = 298.15;  //room temperature in Kelvins
float roomR = 10000;   //thermistor resistance at room temperature
float R = 10000;   //10k resistor used in voltage divider
float B = 3950;   //thermistor coefficient
int i = 0;

void setup() {
  Serial.begin(9600);
  delay(70);  //70 ms setting time
}

void loop() {
  adcReading = (float)analogRead(TPIN);
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
//  if (i > 0){
//    Serial.print("Resistance: ");
//    Serial.print(tResistance);
//    Serial.print("\t\tTemperature: ");
//    //Serial.print(cTemp);
//    Serial.print(fTemp);
//    Serial.print("\n");  
//  }
  if (i == 0){
    i++;
  }
  
  reading = (float)analogRead(HPIN);
  //ADC reading to voltage conversion
  Vout = (reading*Vin)/1023.0;
  //Max voltage adjustment
  maxAdjust = maxV - (0.0062 * cTemp);
  //Datasheet transfer function
  H = ((Vout - minV)/maxAdjust) * 100;
  //H = ((Vout - 0.958)/0.0307) * 100;
  //H = 161.0 * Vout / Vin - 25.8;
  //RH = H/(1.0456 - 0.0026 * 11.11);

  Serial.print("Humidity: ");
  Serial.print(H, 1);
  Serial.print("%\t Vout: ");
  Serial.println(Vout);

  delay(5500);    //5 second response time stated in datasheet
}
