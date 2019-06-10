
#include <math.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHDatagram.h>
#include <RHReliableDatagram.h>

//Sampling interval (in minutes). Adjust to whichever value necessary
#define intervalPeriod 10

//Adjust once actual values are found
#define minPSI 1.5    //min. pressure value to indicate valve is open
#define minBrightness 250   //min. photoresistor resistance to indicate UV LED is on 
//Battery level conditions
#define minPercent 10   //min. battery percentage for system execution
#define replaceCheck 40 //battery percentage needs to be 40% or higher to exit low battery state
//Radio setup
#if defined(__AVR_ATmega2560__)//__AVR_ATmega2560__ for mega and __AVR_ATmega328P__
#define RFM95_CS   53   // MEGA = 53, UNO=4
#define RFM95_RST  4   // MEGA= 4, UNO=2
#define RFM95_INT  2   // MEGA = 2 UNO=3
#endif
//Humidity sensor constants
#define voltDrop 0.0062    //as temp increases, Vout drops by this amount
#define maxV 3.9    //nominal max. voltage output
#define minV 0.7    //minimum voltage output at any temperature

//LCD Display Initialization
/* Detailed pin configuration for LCD
   Pin1 (VSS) = GND
   Pin2 (VDD) = power
   Pin3 (Vo) = brown resistors on PCB
   Pin4 (RS) = pin 33
   Pin5 (R/W) = GND
   Pin6 (E) = pin 3
   Pin11 (DB4) = pin 31
   Pin12 (DB5) = pin 29
   Pin13 (DB6) = pin 27
   Pin14 (DB7) = pin 25
   Pin15 (LED+) = blue resistor on PCB
   Pin16 (LED-) = GND
   Pressure sensor(side with writing faces right): Blue(outermost), Purple, Green(innermost)
*/
const int rs = 33, en = 34, d4 = 31, d5 = 29, d6 = 27, d7 = 25;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//State machine states
enum STATES {OFF, SLEEP, LEAK, LowBattery, AWAKE, IsValveOpen, ValveError, INTAKE,
             IsUVLedOn, UVLedError, READ, errorState
            };
uint8_t state = OFF;  //initial state

//Device pin configuration
int resistancePin = A5;      //resistance sensor pin
int TempPin  =      A1;      //temperature sensor pin
int HumidityPin =   A2;      //humidity sensor pin
int PressurePin =   A3;      //pressure sensor pin
int IntensityPin =  A4;      //light intensity pin
int buttonPin =     6;      //button output pin
int valvePin =      30;      //valve pin
int ledPin =        7;     //UV LED mosfet pin
int batteryPin =    A0;   //battery signal pin
int sensorSwitch =  33;
int radioSwitch =   34;

//Time variables
int timer; //main timer
int saveTime; //time saved when a state is entered
int timeDiff; //difference between the two times
long msTime; //time in milliseconds

float PSI;    //pressure reading
int brightness, H, T, battery;  //light intensity, humidity, temperature, battery reading
int confirmation = 0;       //transceiver confirmation
float humidityOut, pressureOut, tempOut, lightOut, resistanceOut, batteryOut;  //analog readings

//Button variables
int buttonFlag, buttonOut, setFlag;
boolean pressed = false;

//Transceiver variables
// Frequency must match RX's frequncy
#define RF95_FREQ 915.0
//Singleton instance of the radio driver
//RH_RF95 rf95(RFM95_CS, RFM95_INT);
char NODE_ADDRESS[2] = "1";   //Upon initializing a new node, enter a number as the address (1,2,3,4...9).
char UNIVERSALPING[2] = "R";  //if an R is recieved, send signal to take sample
int x;
int messageFlag = 0;
int month = 3;
int day = 18;
int year = 19;
int Hour = 14;
int mnte = 58;
int unknownR;
//int unknownR[5];
int ppm = 83;
char radiopacket[200];
int checksm = 0;
int packetLength, i, j, csLength;
char test[10];
int receiveFlag = 0;

//Error state variables
int errorFlag = 0;

void setup() {
  Serial.begin(115200);
  //LCD setup
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  lcd.clear();
  //Switch, button, LED, & valve setup
  pinMode(buttonPin, INPUT);
  pinMode (valvePin, OUTPUT);
  pinMode(sensorSwitch, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(valvePin, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(sensorSwitch, LOW);
  /*
    //Transceiver setup
    Serial.print("NODE Reciever powered on, node ");
    Serial.println((char*) NODE_ADDRESS);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, LOW);
    //Manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    //Waits for the radio to be initialized, essentially everything is hooked up correctly and the chip isnt broken
    while (!rf95.init()) {
    Serial.println("RFM95 radio initialization failed (Somethings Wrong #4)");
    while (1);
    }
    Serial.println("RFM95 radio initialization success!");
    if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed (Somethings Wrong #5)");
    while (1);
    }
  */
  /* The default transmitter power is 13dBm, using PA_BOOST.
     If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
     you can set transmitter powers from 5 to 23 dBm:
  */
  //rf95.setTxPower(23, false);
  //Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");
}

void loop() {
  timer = millis(); //retrieves time in milliseconds

  char testpacket[25] = "1 Test Packet";                //This needs to be specfied ie "NODEADRESS Test Packet"
  char acknowledge[40] = "Acknowledge test packet";
  char uniacknowledge[40] = "Signaling Microcontroller Sample";
  char data[100];

  //Check message flags for error state
  /*
    //Checks to see if receiver is available
    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      //Checks to see if signal was received from transceiver
      if (rf95.recv(buf, &len)) {
        buf[len] = {0}; // zero out remaining string
        Serial.print("Got packet from node");
        Serial.print(" [RSSI :");
        Serial.print(rf95.lastRssi());
        Serial.print("] : ");
        Serial.println((char*)buf);

        if ((strcmp(testpacket, (char*)buf)) == 0 ) {
          rf95.send(acknowledge, sizeof(acknowledge));
          rf95.waitPacketSent();
          Serial.println("Sent a reply");
        } else if ((strcmp(NODE_ADDRESS, (char*)buf)) == 0) {
          Serial.print("this should be 0: "); Serial.println(x);
          rf95.send(data, sizeof(data));
          rf95.waitPacketSent();
          Serial.println("Sent a reply");
        } else if ((strcmp(UNIVERSALPING, (char*)buf)) == 0) {
          Serial.println("Taking Sample, signalling to microcontroller");
          rf95.send(uniacknowledge, sizeof(uniacknowledge));
          rf95.waitPacketSent();
          Serial.println("Sent a reply");
          receiveFlag = 1;
        }  else {
          Serial.println("Message recieved, not addressed to us");
        }
      } else {     //Send a reply back to the originator client
        Serial.println("Sending failed (no ack)");
      }
    }
  */
  switch (state) {
    case OFF:
      LCDDisplay(0, 0, 0, 2, 0);
      //Second to millisecond interval conversion
      msTime = (intervalPeriod * 60) * 1000; //conversion from minutes to milliseconds for timing
      state = SLEEP;
      saveTime = timer;
      //Battery percentage reading
      batteryOut = analogRead(batteryPin);
      battery = batteryLevel(batteryOut);
      Serial.println("Moving to SLEEP");
      LCDDisplay(0, 0, 0, 4, battery);   //displays sleep mode message
      break;

    case SLEEP:
      //Clears radio packet array
      for (i = 0; i < 200; i++) {
        radiopacket[i] = 0;
      }
      timeDiff = timer - saveTime;
      //Checks the battery level every 5 minutes during sleep
      if (timeDiff % 1000 == 0) {   //changed from 120000 to 1000 for testing purposes
        //Battery Level Reading
        batteryOut = analogRead(batteryPin);
        battery = batteryLevel(batteryOut);
        //Serial.print("Battery: ");
        //Serial.println(battery);
      }
      //Testing: Prints once move on time is reached
      if ((timeDiff % 20000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Checks interval period between samples is over
      //if (timeDiff >= msTime){
      if (timeDiff >= 10000) {    //changed time from 600,000 (10 minutes) to 5,000 for testing purposes
        LCDDisplay(0, 0, 0, 3, 0);   //displays sampling message on LCD
        digitalWrite(valvePin, HIGH);   //opens valve
        digitalWrite(sensorSwitch, HIGH); //turn on pressure sensor
        Serial.println("Moving to IsValveOpen");
        state = IsValveOpen;
        saveTime = timer;
      }
      //Checks receive flag status
      if (receiveFlag == 1) {
        LCDDisplay(0, 0, 0, 3, 0);   //displays sampling message on LCD
        digitalWrite(valvePin, HIGH);   //opens valve
        digitalWrite(sensorSwitch, HIGH); //turn on sensors
        Serial.println("Moving to IsValveOpen");
        state = IsValveOpen;
        saveTime = timer;
        receiveFlag = 0;  //resets flag
      }
      //Button signal check
      buttonOut = digitalRead(buttonPin);
      buttonFlag = ButtonPress(buttonOut);
      //Checks to see if the button was pressed
      if ((pressed == 0) && (buttonFlag == HIGH)) {
        pressed = 1;
      }
      //Checks if button was released
      if ((pressed == 1) && (buttonFlag == LOW)) {
        setFlag = 1;
        pressed = 0;    //reset flag once the button is released
      }
      //Checks if the button flag is high
      if (setFlag == 1) {
        digitalWrite(sensorSwitch, HIGH);
        Serial.println("Moving to AWAKE");
        //Humidity sensor reading
        humidityOut = analogRead(HumidityPin);
        H = HumidityReading(humidityOut);
        //Temperature sensor reading
        tempOut = analogRead(TempPin);
        T = TempReading(tempOut);
        //Pressure sensor reading
        pressureOut = analogRead(PressurePin);
        PSI = PressureReading(pressureOut);
        LCDDisplay(H, PSI, T, 1, 0);   //displays sensor info on LCD
        state = AWAKE;
        saveTime = timer;
        setFlag = 0;    //resets button flag
      }
      //Checks if battery percentage is less than 10%
      if (battery <= minPercent) {
        state = errorState;
        Serial.println("Moving to errorState. Low battery");
        saveTime = timer;
        errorFlag = 3;
        LCDDisplay(0, 0, 0, 6, 0);   //displays sleep mode message
      }
      /*
        //Pressure reading
        //pressureOut = analogRead(PressurePin);
        //Checks if chamber is not airtight
        if (pressureOut !PSI range) {
        state = LEAK;
        saveTimer = timer;
        }0
      */
      break;

    case LEAK:
      //send leak message
      /*
        timeDiff = timer - saveTime;
        if (timeDiff % 2000 == 0){
          //pressureOut = analogRead(PressurePin);
          PSI = PressureReading(pressureOut);
        }
        //Check if pressure range went back to normal
        if (PSI is in expected PSI range when chamber is airtight){
          state = SLEEP;
          saveTimer = timer;
        }
      */
      break;

    case LowBattery:
      //send low battery message
      batteryOut = analogRead(batteryPin);
      battery = batteryLevel(batteryOut);
      //Checks to see if battery was replaced
      if (battery > minPercent) {
        state = SLEEP;
        saveTime = timer;
        //Battery percentage reading
        batteryOut = analogRead(batteryPin);
        battery = batteryLevel(batteryOut);
        Serial.println("Moving to SLEEP");
        LCDDisplay(0, 0, 0, 4, battery);   //displays sleep mode message
      }
      break;

    case AWAKE:
      timeDiff = timer - saveTime;
      //Testing: Displays message once move on time is reached
      if ((timeDiff % 10000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Checks if 10 seconds have passed by to move back to SLEEP
      if (timeDiff >= 10000) {
        lcd.clear();    //clears sensor info from LCD
        digitalWrite(sensorSwitch, LOW); //turn off sensors through switch
        //Battery percentage reading
        batteryOut = analogRead(batteryPin);
        battery = batteryLevel(batteryOut);
        LCDDisplay(0, 0, 0, 4, battery);    //displays sleep mode message
        Serial.println("Moving to SLEEP");
        state = SLEEP;
        saveTime = timer;
      }
      //Button signal reading
      buttonOut = digitalRead(buttonPin);
      buttonFlag = ButtonPress(buttonOut);
      //Checks to see if button was pressed
      if ((pressed == 0) && (buttonFlag == HIGH)) {
        pressed = 1;
      }
      //Checks to see if the button was let go
      if ((pressed == 1) && (buttonFlag == LOW)) {
        setFlag = 1;
        pressed = 0;    //resets flag once button is released
      }
      //Checks to see if the button flag is high
      if (setFlag == 1) {
        lcd.clear();    //clears LCD display
        LCDDisplay(0, 0, 0, 3, 0);   //displays sampling message to LCD
        digitalWrite(valvePin, HIGH);   //opens valve
        Serial.println("Please wait. Sampling will soon begin");
        Serial.println("Moving to IsValveOpen.");
        state = IsValveOpen;
        saveTime = timer;
        setFlag = 0;  //resets button flag
      }
      break;

    case IsValveOpen:
      timeDiff = timer - saveTime;
      //Testing: Prints that move on time was reached
      if ((timeDiff % 4000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Pressure Reading
      //pressureOut = analogRead(PressurePin);
      //PSI = PressureReading(pressureOut);
      PSI = 3.0;    //testing code**********
      //Checks to see if valve was opened by comparing current pressure sensor value to unopen valve pressure reading
      if (PSI > minPSI) {   //update vale in condition once known
        state = INTAKE;
        Serial.println("\tExpected pressure range");
        Serial.println("Moving to INTAKE");
        saveTime = timer;
      } else if (timeDiff >= 5000) {    //valve error timeout
        Serial.println("\tNot the expected pressure range");
        Serial.println("Moving to ValveError");
        //turn off sensor array through switch
        state = ValveError;
        saveTime = timer;
      }
      break;

    case ValveError:
      errorFlag = 1;    //sets error flag
      state = errorState;
      saveTime = timer;
      LCDDisplay(0, 0, 0, 7, 0);   //displays sleep mode message
      break;

    case INTAKE:
      timeDiff = timer - saveTime;
      //Testing: Move on time check
      if ((timeDiff % 10000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Pressure Reading
      //pressureOut = analogRead(PressurePin);
      //PSI = PressureReading(pressureOut);
      //Checks if ten seconds have passed by or PSI is greater than 3
      if ((timeDiff >= 10000) /*| (PSI > 3.0)*/) {
        digitalWrite(valvePin, LOW);    //close valve
        digitalWrite(ledPin, HIGH);     //turn on UV LED
        state = IsUVLedOn;
        Serial.println("Moving to IsUVLedOn");
        saveTime = timer;
      }
      break;

    case IsUVLedOn:
      timeDiff = timer - saveTime;
      //Light intensity reading
      //lightOut = analogRead(IntensityPin);
      //brightness = LightIntensity(lightOut);
      brightness = 300;   //testing**************************
      timeDiff = timer - saveTime;
      //Checks to see if UV LED was turned on by comparing it to the light intensity when UV LED isn't turned on
      if (brightness >= minBrightness) {
        state = READ;
        Serial.println("\tExpected light intensity");
        Serial.println("Moving to READ");
        saveTime = timer;
      } else if (timeDiff >= 5000) {   //UV LED error timeout
        confirmation = 0;
        digitalWrite(ledPin , LOW);   //turns off UV LED
        Serial.println("\tNot txpected light intensity");
        Serial.println("Moving to UVLedError");
        //turn off sensor array
        state = UVLedError;
        saveTime = timer;
      }
      break;

    case UVLedError:
      errorFlag = 2;    //sets error flag
      state = errorState;
      saveTime = timer;
      LCDDisplay(0, 0, 0, 8, 0);   //displays sleep mode message
      break;

    case READ:
      timeDiff = timer - saveTime;
      //Zinc-oxide sensor resistance readings
      resistanceOut = analogRead(resistancePin);
      unknownR = ResistanceMeasure(resistanceOut);
      //Recording resistance
      //      for (i = 0; i < 10; i++) {
      //        resistanceOut = analogRead(resistancePin);
      //        unknownR[i] = ResistanceMeasure(resistanceOut);
      //      }
      //Testing: Move on time check
      if ((timeDiff % 5000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Checks if 5 seconds of data recording has occurred
      if (timeDiff >= 5000) {
        //Humidity sensor reading
        humidityOut = analogRead(HumidityPin);
        H = HumidityReading(humidityOut);
        //Temperature sensor reading
        tempOut = analogRead(TempPin);
        T = TempReading(tempOut);
        //Pressure sensor reading
        pressureOut = analogRead(PressurePin);
        PSI = PressureReading(pressureOut);
        sprintf(data, "H:%d T:%d R:%d dT P:%d", H, T, unknownR, (int)PSI);  //data string to be sent
        Serial.println(data);
        digitalWrite(ledPin, LOW);   //turns off UV LED
        state = SLEEP;
        Serial.println("Moving to SLEEP");
        saveTime = timer;
        LCDDisplay(0, 0, 0, 4, battery);   //displays sleep mode message
      }
      break;

    case errorState:
      timeDiff = timer - saveTime;
      if (timeDiff >= 60000) {
        lcd.clear();
        Serial.println("Error timeout. Moving back to SLEEP");
        state = SLEEP;
        errorFlag = 0;
        saveTime = timer;
      }
      //Checks what triggered the error state
      if (errorFlag == 1) {  //valve error flag
        messageFlag = 1;
        errorFlag = 0;   //resets flag
      } else if (errorFlag == 2) { //UV LED error flag
        messageFlag = 2;
        errorFlag = 0;   //resets flag
      } else if (errorFlag == 3) { //low battery flag
        messageFlag = 3;
        state = LowBattery;
        errorFlag = 0;   //resets flag
      }
      break;
  }
}


//Returns the output of the pressure sensor
float PressureReading(float AD) {
  float kPa, PSI;
  /*  Datasheet transer function: P = (1/0.0018)*(Vout/Vin - 0.04)
       Vout/Vin = AD/1023
  */
  kPa = ((AD / 1023.0) - 0.04) * (1.0 / 0.0018);
  //0.145 PSI = 1.0 kPa
  PSI = kPa * 0.145;
  return abs(PSI);
}

//Returns the output of the temperature sensor
int TempReading(float adcReading) {
  int fTemp, cTemp, kTemp;   //Fahrenheit, Celsisus, Kelvin
  float inverseTemp;    //result of simplified Steinhart-Hart eqn.
  float tResistance;  //thermal resistance
  float roomT = 298.15;  //room temperature in Kelvins
  float roomR = 10000.0;   //thermistor resistance at room temperature
  float R = 10000.0;   //10k resistor used in voltage divider
  float B = 3950.0;   //thermistor coefficient
  //tResistance = (R * ((1023.0 / adcReading) - 1)) - 120.0;
  tResistance = (R*(1023.0-adcReading))/(adcReading);
  //Resistance to Temperature Conversion
  inverseTemp = log(tResistance / roomR);
  inverseTemp = inverseTemp / B;
  inverseTemp = (1.0 / roomT) + inverseTemp;
  kTemp = 1.0 / inverseTemp;
  cTemp = kTemp - 273.15;       //converts from Kelvin to Celsius
  fTemp = (int)(((cTemp * 1.8) + 32.0) - 8.0);   //converts from Celsius to Fahrenheit
  return fTemp;
}

//Returns the output of the humidity sensor
int HumidityReading(float reading) {
  //delay(500);
  float Vout, maxAdjust, cTemp;
  int H;
  Vout = (reading * 5.0) / 1023.0;    //ADC reading to voltage conversion
  cTemp = TempReading(TempPin);   //retrieves temperature
  maxAdjust = maxV - (voltDrop * cTemp);  //max. output voltage calibration
  H = (int)(((Vout - minV) / maxAdjust) * 100);  //datasheet transfer function
  return H;
}

//Returns the output of the light intensity sensor
int LightIntensity(int reading) {
  int rLight;
  int R = 1000.0;   //resistor used in voltage divider
  //Rearranged voltage divider equation
  rLight = R * ((1023.0 / reading) - 1.0); //resistance decreases as LI increases
  return rLight;
}

//Returns the unknown resistance
int ResistanceMeasure(float raw) {
  int Vin = 5;
  float Vout = 0;
  float R1 = 7410;
  float R2 = 0;
  float buffer = 0;
  //Checks to see if resistance was recorded
  if (raw) {
    //Voltage divider equation breakdown
    buffer = raw * Vin;
    Vout = (buffer) / 1024.0;
    buffer = (Vin / Vout) - 1.0;
    R2 = R1 * buffer;
  }
  return R2;
}

//Returns the output of the button sensor
int ButtonPress(int x) {
  int pressFlag;
  //Checks to see if button was pressed
  if (x == 1) {
    pressFlag = 1;
  } else {
    pressFlag = 0;
  }
  return pressFlag;
}

//Returns the battery percentage
float batteryLevel(float reading) {
  //Voltage divider resistors
  float R1 = 9938.0;
  float R2 = 5049.0;
  //Battery levels
  float voltOffset = 0.9;
  float maxVx = 0.6;
  float vx, vin, percent;
  
  vx = (reading * 5.0) / 1023.0;   // vout/vin = ADC/1023
  //vin = vx * ((R1 + R2) / R2);     //voltage divider equation rearranged in terms of vin
  percent = ((vx - voltOffset) / maxVx) * 100;  //battery level mapping
  return percent;
}

//LCD Printing
void LCDDisplay(int hum, float pressure, float temp, int WAIT, int batteryRead) {
  //Flag for AWAKE (sensor readings)
  if (WAIT == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    //Print humidity
    lcd.print("H:");
    lcd.print(hum);
    lcd.print("% ");
    //Print pressure
    lcd.print("P: 0.8");
    //lcd.print(pressure);
    lcd.print(" PSI");
    //Print temperature
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(temp);
    lcd.print(" F");
  } else if (WAIT == 3) {   //Flag for sampling in process message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Sampling in process. Please wait.");
  }
  else if (WAIT == 4) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Currently in sleep.");
    lcd.setCursor(0, 1);
    lcd.print("Battery: ");
    lcd.print(batteryRead);
    lcd.print("%");
  } else if (WAIT == 6) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Battery percentage low.");
    lcd.setCursor(0, 1);
    lcd.print("Replace in order to proceed.");
  } else if (WAIT == 7) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Valve did not open properly.");
    lcd.setCursor(0, 1);
    lcd.print("Refer to manual to fix issue.");
  } else if (WAIT == 8) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("UV LEDs not working properly.");
    lcd.setCursor(0, 1);
    lcd.print("Fix to ensure reliable data.");
  }
  return;
}
