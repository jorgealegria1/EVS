//Add please wait message if button is pressed outside of SLEEP and AWAKE
/*
       For previous updates, read the comment at the top of the old version

       Version 1.4- Updated transceiver state to send data
          -Sending error messages for valves and UV LED
*/

#include <math.h>
#include <LiquidCrystal.h>
//#include <SD.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

//LCD Display Initialization
const int rs = 22, en = 13, d4 = 12, d5 = 24, d6 = 11, d7 = 26;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//SD file declaration
//File myFile;
//File ppm_data;

enum STATES {OFF, InitError, SLEEP, LEAK, LowBattery, AWAKE, IsValveOpen, ValveError, INTAKE,
             IsUVLedOn, UVLedError, READ, TRANSMIT, TransmitError
            };
uint8_t state = OFF;

//Adjust once actual values are found
#define minPSI 1.5
#define minBrightness 100

//Battery level conditions
#define minPercent 10

//Pin Configuration
int resistancePin = A0;      //pressure sensor pin
int TempPin  =      A1;      //temperature sensor pin
int HumidityPin =   A2;      //humidity sensor pin
int PressurePin =   A3;      //pressure sensor pin
int IntensityPin =  A4;      //light intensity pin
int buttonPin =     6;      //button output pin
int servo =      3;      //motor pin
int ledPin =        8;     //LED output pin
//int pinCS =         53;     //SD card pin

/************ Radio Setup ***************/
#if defined(__AVR_ATmega2560__)//__AVR_ATmega2560__ for mega and __AVR_ATmega328P__
#define RFM95_CS   53   // MEGA = 53, UNO=4
#define RFM95_RST  4   // MEGA= 4, UNO=2
#define RFM95_INT  2   // MEGA = 2 UNO=3
#endif

//Humidity sensor constants
#define voltDrop 0.0062    //as temp increases, Vout drops by this amount
#define maxV 3.9
#define minV 0.7

//Time variables
int timer; //main timer
int saveTime; //time saved when a state is entered
int timeDiff; //difference between the two times

float PSI;    //pressure reading
int brightness, H, T;  //light intensity, humidity, temperature reading
int battery;                //battery percentage
int confirmation = 0;       //transceiver confirmation
float humidityOut, pressureOut, tempOut, lightOut, resistanceOut;  //analog readings

//Button variables
int buttonFlag, buttonOut, setFlag;
boolean pressed = false;

//Transceiver variables
// Frequency must match RX's frequncy
#define RF95_FREQ 915.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission
int16_t TestSuccess = 0;
int16_t TestFail = 0;
int16_t Fail = 0;
char Transmit = 0;

//Transceiver variables
int month = 3;
int day = 18;
int year = 19;
int Hour = 14;
int mnte = 58;
int unknownR;
//int unknownR[5];
int ppm = 83;
int previousTime = 0;   //timer for LED blink
int blinkState = 0;
char radiopacket[200];

//Motor variables
int highDelay = 700;
int lowDelay;
int angle = 10;
int Loop = 20000;

//Error state variables
int v1, v2;

void setup() {
  Serial.begin(115200);
  //LCD setup
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  //Button & LED setup
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode (servo, OUTPUT);   //motor setup
  //pinMode(pinCS, OUTPUT);
  // SD Card Initialization
  //  if (SD.begin())
  //  {
  //    Serial.println("SD card is ready to use.");
  //  } else
  //  {
  //    Serial.println("SD card initialization failed");
  //    return;
  //  }
  //Transceiver setup
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  Serial.print("Transmitter powered on, node 1.");
  Serial.println();

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) { //waits for the radio to be initialized, essentially everything is hooked up correctly and the chip isnt broker
    Serial.println("RFM95 radio initialization failed (Somethings Wrong #4)");
    while (1);
  }
  Serial.println("RFM95 radio initialization success!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed (Somethings Wrong #5)");
    while (1);
  }
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");
}

void loop() {
  timer = millis(); //retrieves time in milliseconds

  switch (state) {
    case OFF:
      LCDDisplay(0, 0, 0, 2);
      while ((TestSuccess < 3) && (TestFail < 10)) {
        delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

        char testradiopacket[20] = "Test packet #"; //This will need to be a dummy variable for the test portion
        itoa(packetnum++, testradiopacket + 13, 10);

        Serial.print("Sending "); Serial.println(testradiopacket);
        delay(10);
        rf95.send((uint8_t *)testradiopacket, 20); // Send a message to the DESTINATION!
        rf95.waitPacketSent();

        rf95.waitPacketSent();
        // Now wait for a reply
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.waitAvailableTimeout(1000))
        {
          // Now wait for a reply from the server, Should be a reply message for us now
          if (rf95.recv(buf, &len)) {
            Serial.print("Got reply. ");
            Serial.print(" [RSSI :");
            Serial.print(rf95.lastRssi());
            Serial.print("] : ");
            Serial.println((char*)buf);
            TestSuccess = TestSuccess + 1;
            if (rf95.lastRssi() < (-90)) {
              Serial.println("Signal strength below -90dbm (Somethings Wrong #1)");
            }
          } else {
            Serial.println("Recieve failed (no ack) (Somethings Wrong #3)");
            TestFail = TestFail + 1;
            Serial.print("Test Fail "); Serial.print(int(TestFail)); Serial.println(" times");
          }
        } else {
          Serial.println("No reply, is anyone listening? (Somethings Wrong #2)");
          TestFail = TestFail + 1;
          Serial.print("Test Fail "); Serial.print(int(TestFail)); Serial.println(" times");
        }
      }
      //Checks if initialization was successful
      if (TestSuccess == 3 ) {
        Serial.println ("connected :)");
        Serial.println ("");
        TestSuccess = TestSuccess + 1;
        Transmit = Transmit + 1; //This is for testing the code to send it into transmition, typically that will come from the microcontroller
        //output to LED connected symbol
        state = SLEEP;
        Serial.println("Moving to SLEEP");
        saveTime = timer;   //stores time when state is entered
      }
      //Checks if initialization failed
      if (TestFail == 10 ) {
        Serial.println ("Move Node, connection failed. Moving to InitError");
        state = InitError;
        saveTime = timer;
        TestFail = 0;
        //******may need to output an error message to the LCD
      }
      break;

    case InitError:
      timeDiff = timer - saveTime;
      if ((timeDiff % 5000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached. Trying initialization again");
      }
      //Checks if 5 seconds have passed by
      if (timeDiff >= 5000) {
        state = OFF;
        Serial.println("Trying initialization again");
      }
      break;

    case SLEEP:
      LCDDisplay(0, 0, 0, 4);   //displays sleep mode message
      timeDiff = timer - saveTime;
      if ((timeDiff % 20000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Checks if 10 minutes have passed by
      if (timeDiff >= 20000) {    //changed time from 60,000 to 10,000 for testing purposes
        LCDDisplay(0, 0, 0, 3);   //displays sampling message
        //open valve
        //turn on pressure sensor
        Serial.println("Moving to IsValveOpen");
        //Servo movement
        highDelay = highDelay + angle; // increasing angle to move servo
        digitalWrite(servo, HIGH);
        delayMicroseconds(highDelay);
        lowDelay = Loop - highDelay ;
        digitalWrite(servo, LOW);
        delayMicroseconds(lowDelay);
        delay (10);
        if ( highDelay >= 2450 || highDelay <= 700 ) {
          angle = -angle ;
        }
        delay(50);
        state = IsValveOpen;
        saveTime = timer;
      }
      //Checks to see if button was pressed
      buttonOut = digitalRead(buttonPin);
      buttonFlag = ButtonPress(buttonOut);
      //Checks if button was pressed
      if ((pressed == 0) && (buttonFlag == HIGH)) {
        pressed = 1;
      }
      //Checks if button was let go
      if ((pressed == 1) && (buttonFlag == LOW)) {
        setFlag = 1;
        pressed = 0;    //reset flag once button is released
      }
      //Checks if button flag is high
      if (setFlag == 1) {
        Serial.println("Moving to AWAKE");
        LCDDisplay(0, 0, 0, 5);
        //turn on LCD screen
        //turn on sensors
        state = AWAKE;
        saveTime = timer;
        setFlag = 0;  //reset flag
      }
      /*
        //battery percentage reading
        //Checks if battery percentage is less than 10%
        if (battery = < minPercent) {
        state = LowBattery;
        }
        //Pressure reading
        //pressureOut = analogRead(PressurePin);
        //Checks if chamber is not airtight
        if (pressureOut !PSI range) {
        state = LEAK;
        }
      */
      break;

    case LEAK:
      //send leak message
      //Check if pressure range went back to normal
      /*
        //pressureOut = analogRead(PressurePin);
        if (pressureOut in PSI range){
        state = SLEEP;
        }
      */
      break;

    case LowBattery:
      //send low battery message
      //Checks if battery was replaced
      /*
        if (battery > minPercent) {
        state = SLEEP;
        }
      */
      break;

    case AWAKE:
      timeDiff = timer - saveTime;
      //testing
      if ((timeDiff % 10000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Analog readings
      humidityOut = analogRead(HumidityPin);
      tempOut = analogRead(TempPin);
      pressureOut = analogRead(PressurePin);
      //Sensor readings
      H = HumidityReading(humidityOut);
      PSI = PressureReading(pressureOut);
      T = TempReading(tempOut);
      LCDDisplay(H, PSI, T, 1);   //displays info on LCD
      //Checks if 10 seconds passed by
      if (timeDiff >= 10000) {
        lcd.clear();    //clears LCD display
        //turn off sensors
        //turn off LCD display
        Serial.println("Moving to SLEEP");
        state = SLEEP;
        saveTime = timer;
      }
      //Checks to see if button was pressed
      buttonOut = digitalRead(buttonPin);
      buttonFlag = ButtonPress(buttonOut);
      if ((pressed == 0) && (buttonFlag == HIGH)) {
        pressed = 1;
      }
      //Checks to see if button was let go
      if ((pressed == 1) && (buttonFlag == LOW)) {
        setFlag = 1;
        pressed = 0;    //resets flag once button is released
      }
      //Checks to see if button flag is high
      if (setFlag == 1) {
        lcd.clear();  //clears LCD display
        //open valve
        //turn on pressure sensor
        LCDDisplay(0, 0, 0, 3);   //displays please wait
        //Servo movement
        highDelay = highDelay + angle; // increasing angle to move servo
        digitalWrite(servo, HIGH);
        delayMicroseconds(highDelay);
        lowDelay = Loop - highDelay ;
        digitalWrite(servo, LOW);
        delayMicroseconds(lowDelay);
        delay (10);
        if ( highDelay >= 2450 || highDelay <= 700 ) {
          angle = -angle ;
        }
        delay(50);
        Serial.println("Please wait. Sampling will soon begin");
        Serial.println("Moving to IsValveOpen.");
        state = IsValveOpen;
        saveTime = timer;
        setFlag = 0;  //reset flag
      }
      break;

    case IsValveOpen:
      timeDiff = timer - saveTime;
      //Testing
      if ((timeDiff % 4000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Pressure Reading
      //pressureOut = analogRead(PressurePin);
      //PSI = PressureReading(pressureOut);
      PSI = 3.0;    //testing code**********
      //Checks to see if valve was opened by comparing pressure sensor value to the value when gas isn't being taken in
      if (PSI > minPSI) {
        state = INTAKE;
        Serial.println("\tExpected pressure range");
        Serial.println("Moving to INTAKE");
        saveTime = timer;  //stores time when state was entered
      } else if (timeDiff >= 4000) {
        Serial.println("\tNot the expected pressure range");
        Serial.println("Moving to ValveError");
        //turn off pressure sensor
        state = ValveError;
        saveTime = timer;
      }
      break;

    case ValveError:
      Transmit = 1;
      timeDiff = timer - saveTime;
      if ((Transmit == 1) && (v1 == 0)) { //this will be a global varialbe from the microcontroller
        delay(5000); //transmit every 5 seconds for the prototype, delete after real transmit exists

        sprintf(radiopacket, "Valve error. Valve not working properly");
        //radiopacket = "Valve error.";

        //itoa(packetnum++, radiopacket + 13, 10);
        Serial.print("Sending "); Serial.println(radiopacket);
        rf95.send((uint8_t *)radiopacket, 20);

        rf95.waitPacketSent();
        // Now wait for a reply
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        v1++;

        // Send a message to the DESTINATION!
        if (rf95.waitAvailableTimeout(1000) && !(Fail == 10)) {
          // Now wait for a reply from the server
          uint8_t len = sizeof(buf);
          uint8_t from;
          if (rf95.recv(buf, &len)) {
            buf[len] = 0; // zero out remaining string

            Fail = 0;
            Serial.print("Got reply from HUB");
            Serial.print(" [RSSI :");
            Serial.print(rf95.lastRssi());
            Serial.print("] : ");
            Serial.println((char*)buf);
            confirmation = 1;
            if (rf95.lastRssi() < (-90)) {
              Serial.println("Signal strength below -90dbm (Somethings Wrong #1)");
            }
          }
          else {
            delay(1000);
            Serial.println("Sending failed (no ack) (Somethings Wrong #6)");
            Fail = Fail + 1;
            Serial.print("Test Fail "); Serial.print(int(Fail)); Serial.println(" times");
          }
        }
        else {
          if (Fail < 10) {
            delay(1000);
            Serial.println("Not Sent (Somethings Wrong #7)");
            Fail = Fail + 1;
            Serial.print("Test Fail "); Serial.print(int(Fail)); Serial.println(" times");
          }
        }

        if (Fail == 10) {
          Serial.println ("Send Failed multiple times, connection lost, move node");
          delay(5000);
        }
      }
      //Checks if ten seconds have passed by
      if (timeDiff >= 10000) {
        Serial.println("Moving to SLEEP");
        confirmation = 0;
        state = SLEEP;
        saveTime = timer;
      }
      break;

    case INTAKE:
      timeDiff = timer - saveTime;
      //testing
      if ((timeDiff % 10000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Pressure Reading
      //pressureOut = analogRead(PressurePin);
      //PSI = PressureReading(pressureOut);
      //Checks if ten seconds have passed by or PSI is greater than 3
      if ((timeDiff >= 10000) /*| (PSI > 3.0)*/) {
        //close valve
        //Servo movement
        highDelay = highDelay + angle; // increasing angle to move servo
        digitalWrite(servo, HIGH);
        delayMicroseconds(highDelay);
        lowDelay = Loop - highDelay ;
        digitalWrite(servo, LOW);
        delayMicroseconds(lowDelay);
        delay (10);
        if ( highDelay >= 2450 || highDelay <= 700 ) {
          angle = -angle ;
        }
        delay(50);
        digitalWrite(ledPin, HIGH);
        //turn on light intensity sensor
        //turn off pressure sensor
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
      brightness = 200;   //testing**************************
      timeDiff = timer - saveTime;
      //Checks to see if UV LED was turned on by comparing it to the light intensity when it isn't turned on
      if (brightness >= minBrightness) {
        state = READ;
        Serial.println("\tExpected light intensity");
        Serial.println("Moving to READ");
        saveTime = timer;
      } else if (timeDiff >= 10000) {
        confirmation = 0;
        Serial.println("\tNot txpected light intensity");
        Serial.println("Moving to UVLedError");
        //turn off light intensity sensor
        state = UVLedError;
        saveTime = timer;
      }
      break;

    case UVLedError:
      Transmit = 1;
      timeDiff = timer - saveTime;
      if ((Transmit == 1) && (v1 == 0)) { //this will be a global varialbe from the microcontroller
        delay(5000); //transmit every 5 seconds for the prototype, delete after real transmit exists

        sprintf(radiopacket, "UV LED error #010");

        //itoa(packetnum++, radiopacket + 13, 10);
        Serial.print("Sending "); Serial.println(radiopacket);
        rf95.send((uint8_t *)radiopacket, 20);

        rf95.waitPacketSent();
        // Now wait for a reply
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        v1++;

        // Send a message to the DESTINATION!
        if (rf95.waitAvailableTimeout(1000) && !(Fail == 10)) {
          // Now wait for a reply from the server
          uint8_t len = sizeof(buf);
          uint8_t from;
          if (rf95.recv(buf, &len)) {
            buf[len] = 0; // zero out remaining string

            Fail = 0;
            Serial.print("Got reply from HUB");
            Serial.print(" [RSSI :");
            Serial.print(rf95.lastRssi());
            Serial.print("] : ");
            Serial.println((char*)buf);
            confirmation = 1;
            if (rf95.lastRssi() < (-90)) {
              Serial.println("Signal strength below -90dbm (Somethings Wrong #1)");
            }
          }
          else {
            delay(1000);
            Serial.println("Sending failed (no ack) (Somethings Wrong #6)");
            Fail = Fail + 1;
            Serial.print("Test Fail "); Serial.print(int(Fail)); Serial.println(" times");
          }
        }
        else {
          if (Fail < 10) {
            delay(1000);
            Serial.println("Not Sent (Somethings Wrong #7)");
            Fail = Fail + 1;
            Serial.print("Test Fail "); Serial.print(int(Fail)); Serial.println(" times");
          }
        }

        if (Fail == 10) {
          Serial.println ("Send Failed multiple times, connection lost, move node");
          delay(5000);
        }
      }
      //testing
      if ((timeDiff % 10000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //display UV LED didn't turn on error
      if (timeDiff >= 10000) {
        confirmation = 0;
        state = SLEEP;
        Serial.println("Moving to SLEEP");
        saveTime = timer;
      }
      break;

    case READ:
      timeDiff = timer - saveTime;
      resistanceOut = analogRead(resistancePin);
      unknownR = ResistanceMeasure(resistanceOut);
      //Recording resistance
      //      for (i = 0; i < 10; i++) {
      //        resistanceOut = analogRead(resistancePin);
      //        unknownR[i] = ResistanceMeasure(resistanceOut);
      //      }
      //Testing
      if ((timeDiff % 5000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //Checks if 5 seconds haven't passed by
      if (timeDiff < 5000) {
        //log data into array
        //time stamp
      }
      //Checks if 5 seconds passed by
      else if (timeDiff >= 5000) {
        //wake up transceiver
        //display time stamp & display data
        //flush
        //turn off UV LED
        digitalWrite(ledPin, LOW);   //test LED turned off
        state = TRANSMIT;
        Serial.println("Moving to TRANSMIT");
        saveTime = timer;
      }
      break;

    case TRANSMIT:
      Transmit = 1;
      //      //PPM
      //      //send sensor info
      timeDiff = timer - saveTime;
      if ((Transmit == 1)) { //this will be a global varialbe from the microcontroller
        delay(5000); //transmit every 5 seconds for the prototype, delete after real transmit exists

        sprintf(radiopacket, "Resistance: %d", unknownR);

        //itoa(packetnum++, radiopacket + 13, 10);
        Serial.print("Sending "); Serial.println(radiopacket);
        rf95.send((uint8_t *)radiopacket, 20);

        rf95.waitPacketSent();
        // Now wait for a reply
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        // Send a message to the DESTINATION!
        if (rf95.waitAvailableTimeout(1000) && !(Fail == 10)) {
          // Now wait for a reply from the server
          uint8_t len = sizeof(buf);
          uint8_t from;
          if (rf95.recv(buf, &len)) {
            buf[len] = 0; // zero out remaining string

            Fail = 0;
            Serial.print("Got reply from HUB");
            Serial.print(" [RSSI :");
            Serial.print(rf95.lastRssi());
            Serial.print("] : ");
            Serial.println((char*)buf);
            confirmation = 1;
            if (rf95.lastRssi() < (-90)) {
              Serial.println("Signal strength below -90dbm (Somethings Wrong #1)");
            }
          }
          else {
            delay(1000);
            Serial.println("Sending failed (no ack) (Somethings Wrong #6)");
            Fail = Fail + 1;
            Serial.print("Test Fail "); Serial.print(int(Fail)); Serial.println(" times");
          }
        }
        else {
          if (Fail < 10) {
            delay(1000);
            Serial.println("Not Sent (Somethings Wrong #7)");
            Fail = Fail + 1;
            Serial.print("Test Fail "); Serial.print(int(Fail)); Serial.println(" times");
          }
        }

        if (Fail == 10) {
          Serial.println ("Send Failed multiple times, connection lost, move node");
          delay(5000);
        }
      }

      //Checks if 10 seconds were exceeded and no confirmation received
      if ((Fail >= 10)) {
        //sleep transceiver and uC
        Transmit = 0;
        state = TransmitError;
        Serial.println("Moving to TransmitError");
        saveTime = timer;
      }
      else if (confirmation) {
        //sleep transceiver and uC
        Serial.println("Data confirmed. Moving to SLEEP");
        confirmation = 0;
        state = SLEEP;
        saveTime = timer;
      }
      if (timeDiff > 10000) {
        state = TransmitError;
        saveTime = timer;
        Serial.println("Moving to TransmitError");
        digitalWrite(ledPin, HIGH);
      }
      break;

    case TransmitError:
      timeDiff = timer - saveTime;
      //Testing
      if ((timeDiff % 20000 == 0) && (timeDiff != 0)) {
        Serial.println("\tRequired move on time reached");
      }
      //LED blinking for error notification
      if ((timeDiff - previousTime) > 1500) {
        if (blinkState == 0) {
          digitalWrite(ledPin, HIGH);
          previousTime = timeDiff;
          blinkState = 1;
        } else if (blinkState == 1) {
          digitalWrite(ledPin, LOW);
          previousTime = timeDiff;
          blinkState = 0;
        }
      }
      //Checks if ten seconds have passed by
      if (timeDiff >= 20000) {
        state = SLEEP;
        Serial.println("Moving to SLEEP");
        saveTime = timer;
        previousTime = 0;
        digitalWrite(ledPin, LOW);
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
  kPa = (1.0 / 0.0018) * ((AD / 1023.0) - 0.04);
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

  tResistance = ((adcReading * R) / (1023.0 - adcReading)) - 120.0;

  //Resistance to Temperature Conversion
  inverseTemp = log(tResistance / roomR);
  inverseTemp = inverseTemp / B;
  inverseTemp = (1.0 / roomT) + inverseTemp;
  kTemp = 1.0 / inverseTemp;
  cTemp = kTemp - 273.15;       //converts from Kelvin to Celsius
  fTemp = (int)((cTemp * 1.8) + 32.0);   //converts from Celsius to Fahrenheit

  return fTemp;
}

//Returns the output of the humidity sensor
int HumidityReading(float reading) {
  //delay(500);
  float Vout, maxAdjust, cTemp;
  int H;
  Vout = (reading * 5.0) / 1023.0;    //ADC reading to voltage conversion
  cTemp = TempReading(TempPin);   //retrieves temperature
  maxAdjust = maxV - (0.0062 * cTemp);  //max voltage adjustment
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
  if (raw) {
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
  if (x == 1) {
    pressFlag = 1;
  } else {
    pressFlag = 0;
  }
  return pressFlag;
}

//Returns the battery percentage
float batteryLevel(void) {

}

//LCD activation
void LCDDisplay(int hum, float pressure, float temp, int WAIT) {
  if (WAIT == 1) {
    lcd.setCursor(0, 0);
    //Print humidity
    lcd.print("H=");
    lcd.print(hum);
    lcd.print("% ");

    //Print pressure
    lcd.print("P=");
    lcd.print(pressure);
    //lcd.print(" PSI");

    //Print temperature
    lcd.setCursor(0, 1);
    lcd.print("T=");
    lcd.print(temp);
    lcd.print(" F");
  } else if (WAIT == 2) {
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Initialization ongoing. Please wait.");
  }
  else if (WAIT == 3) {
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Sampling in process. Please wait.");
  }
  else if (WAIT == 4) {
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Currently in sleep mode.");
  } else if (WAIT == 5) {
    lcd.clear();
  }
  return;
}

//void sd_log() {
//  //Serial.print(rtc.getTimeStr());
//  //Serial.print(",");
//  Serial.println("R: ");
//  Serial.println(int(unknownR));
//  Serial.print(",");
//  Serial.println("Hum: ");
//  Serial.println(int(H));
//  Serial.print(",");
//  Serial.println("Temp: ");
//  Serial.println(int(T));
//  Serial.print(",");
//  Serial.println("PSI: ");
//  Serial.println(float(PSI));
//
//
//  myFile = SD.open("data.txt", FILE_WRITE);
//  if (myFile) {
//    //myFile.print(rtc.getTimeStr());
//    //myFile.print(",");
//    myFile.println("R: ");
//    myFile.println(int(unknownR));
//    myFile.print(",");
//    myFile.println("Hum: ");
//    myFile.println(int(H));
//    myFile.print(",");
//    myFile.println("Temp: ");
//    myFile.println(int(T));
//    myFile.print(",");
//    myFile.println("PSI: ");
//    myFile.println(float(PSI));
//
//    myFile.close(); // close the file
//  }
//  // if the file didn't open, print an error:
//  else {
//    Serial.println("error opening data.txt");
//  }
//  delay(3000);
//}

