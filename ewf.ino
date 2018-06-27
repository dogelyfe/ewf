//SETUP//
/*
  Hardware
  1. solenoid relay pins
  2. light relay pins
  3. light PWM pins (must be analog)

  Sketch
  1. define proper pins
  2. set times, set pressures, set light %,

*/

//DISPLAY
/*
  Status
    Date
    Time
    Temp
    Humidity

  Configuratble
    Current Time:
    Light On Time
    Light Off Time
    Light Intensity
    Solenoid On Cycle
    Solenoid Off Cycle
    Pump Pressure

  Component States
    Light
    LightIntensity
    Solenoids
    Pump
    Fan
    AC
    Humidifier
*/
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h" //humidity/temp module

#include <DS3231.h> // RTC

#include <ClickEncoder.h>

#include <URTouchCD.h>
#include <URTouch.h> //touchscreen
#include <memorysaver.h>
#include <UTFT.h> //lcd

#include <Chrono.h>
#include <LightChrono.h>

#include <Time.h>
#include <TimeLib.h>

////////////////////////////////
/////////// INITS //////////////
////////////////////////////////

///////// USER DEFINED /////////
bool returnFarenheit = true; //sht31-d F or C
//watering on for millisecs, make adjustable by 1/10ths of sec by pot
unsigned long solenoidOn = 2000;
unsigned long solenoidOff = 5000; //off
unsigned long solenoidOnNight = 1000; //on for millisecs
unsigned long solenoidOffNight = 10000; //off
byte pressureLow = 80;
byte pressureHigh = 125;
byte setLightStartHMS[3] = {00, 00, 10}; // set time lights on HH,MM,SS
byte setLightEndHMS[3] = {00, 00, 15}; // set time lights off HH,MM,SS
byte lightPin[4] = {26, 27, 28, 29}; //light relay pins (digital out) [SET FOR MEGA]
byte lightPWMPin[4] = {9, 10, 11, 12}; //light pwm out pins (analog out) [SET FOR MEGA]
///// PINS /////
byte solenoidPin[] = {22, 23, 24, 25}; // [SET FOR MEGA]
byte pressurePin = 18;
byte pumpPin = 8;

// interrupt0 pin for control pot
byte interruptButtonPin = 2;
/////////////////////////////////
/////////////////////////////////

DS3231  rtc(SDA, SCL);

Chrono printChrono;
Chrono secsChrono(Chrono::SECONDS);

//environment
Adafruit_SHT31 sht31 = Adafruit_SHT31();
//water
bool solenoidState[] = {false, false, false, false};
bool anySolOn = false;
Chrono solenoidChronoOff, solenoidChronoOn[4];
unsigned long solenoidOnTimer, solenoidOffTimer;   //conditional holders for night/day value
//pump
float pressureReading;
float pressurePsi = 90; //starts at 90 to avoid pump instant-on
//time
tmElements_t currentTm, todayTm, tmLightStart, tmLightEnd;
time_t currentT, todayT, tLightStart, tLightEnd;
Time holderTime;

//light
boolean lightState;
//bool lightState[] = {false, false, false, false};
//bool anyLit = false;

byte lightIntensity = 90; //will receive value from pot adjust, should cap at 100
byte lightIntensityLast;

Chrono lightChrono;

////debug
//byte debugReadPin[4] = {97, 96, 95, 94};
//byte debugReading;


///////////////////////////////
/////////// SETUP /////////////
///////////////////////////////

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // while (!Serial) //intended to wait for serial initiate, but seems to pause entire process until serial monitor is up
  //   delay(10);
  rtc.begin();
  //uncomment to set
  // rtc.setDOW(TUESDAY);     // Set Day-of-Week to SUNDAY
  // rtc.setTime(15, 50, 0);     // Set the time to 12:00:00 (24hr format)
  // rtc.setDate(24, 6, 2018);   // day, month, year
  holderTime = rtc.getTime();
  setTime(holderTime.hour,holderTime.min, holderTime.sec, holderTime.date, holderTime.mon, holderTime.year); //(int hr,int min,int sec,int dy, int mnth, int yr)
  currentT = now();
  //interrupts
  pinMode(interruptButtonPin, INPUT_PULLUP);
  // Attach an interrupt to ISR (a function defined after loop), HIGH determines interrupt on button press
  //  attachInterrupt(digitalPinToInterrupt(interruptButtonPin), buttonISR, HIGH);

  //sets variable as pin for analog transducer input (volt)
  //analog in 0-1023
  pinMode(pressurePin, INPUT);
  pinMode(pumpPin, OUTPUT);
  //analogPressurein = (A0)

  //watering
  setSolenoidTiming(); //initializes solenoid timing
  //solenoid relay triggers
  for (int i; i < 4; i++) {
    pinMode(solenoidPin[i], OUTPUT);
  }
  //initiates
  for (int i = 0; i < 4; i++) {
    solenoidChronoOn[i].restart(0 - (i * solenoidOn));
  }
  //pressure
  pinMode(pressurePin, INPUT);

  //lights
  for (int i; i < 4; i++) {
    digitalWrite(lightPin[i], LOW);// check for proper write state if LOW is actually On/off
  }
  for (int i; i < 4; i++) {
    pinMode(lightPin[i], OUTPUT);
  }
  for (int i; i < 4; i++) {
    pinMode(lightPWMPin[i], OUTPUT);
  }

  //  //debug READING
  //  for (int i; i < 4; i++) {
  //    pinMode(debugReadPin[i], INPUT);
  //  }
  //  //END debug

  //needed for tm to work correctly
  tmLightStart.Month = 1;
  tmLightStart.Day = 1;
  tmLightEnd.Month = 1;
  tmLightEnd.Day = 1;

  setLightSchedule();

  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    //    while (1) delay(1);
  }
}

//adjustTime(43200);

///////////////////////////////
/////////// LOOP //////////////
///////////////////////////////

void loop() {
  /*** Status/Readings ***/
  if (printChrono.hasPassed(3000)) {
    Serial.println();
    printTimeReadout();
    printTempHum();
    displaySolenoid();
    displayLight();
  }

  /*** WATERING CYCLE ***/
  //water loop 3.0
  for (int i = 0; i < 4; i++) {
    toggleSolenoidOn(i);
  }
  for (int i = 0; i < 4; i++) {
    toggleSolenoidOff(i);
  }
  //    solenoidChronoOff.restart();

  /*** PUMP CONTROL ***/
  //on at 85 stop at 125?

  pressureReading = analogRead(pressurePin);
  //some code to convert analog reading to psi, and averaging (last 10 measurements) for noise reduction
  //>>>>>needs calibration offsets
  pressurePsi = pressurePsi * 0.9 + map(pressureReading, 102, 922, 0, 200) * 0.1;

  //needs to account for hysteresis/signal noise?
  if (pressurePsi < pressureLow) {
    //pump on
    digitalWrite (pumpPin, LOW);
  }
  if (pressurePsi > pressureHigh) {
    digitalWrite (pumpPin, HIGH);
  }

  /*
    pressurePsi = (pressureReading - offsetPressure) * 1.2 / (fullScale - offset); // pressure conversion
    Serial.print("   The pressure is  ");
    Serial.print(pressure, 3); // three decimal places
    Serial.println("  Mpa");
    delay(500);
  */
  /*** TIME (integrated for Lighting) ***/

  updateLightLoop();
  setLight();

  /*** LIGHTING ***/
  //PWM
  // arduino 5+ --> resistor (1k-10k 4.7K optimal?)(or 1k-100k --> transistor -->dimming dim-
  //battery --> voltage regulator (to transistor emitter) with caps for stability (recommend 20uF on output ) --> dim+
  //https://www.rollitup.org/t/meanwell-led-drivers-3-in-1-dimming-function.838760/page-2
  //may invert duty cycle
  //try without the voltage regulator from battery, since the driver may output its own 10v supply!


  //  analogWrite(lightPWMPin[i],
  if (lightIntensity != lightIntensityLast) {
    for (int i = 0; i < 4; i++) {
      analogWrite(lightPin[i], map(lightIntensity, 0, 100, 0, 255));
      lightIntensityLast = lightIntensity;
    }
  }


  /*** TEMPERATURE ***/
  //


  /*** HUMIDITY ***/


}

///////////////////////////////
////////// FUNCTIONS //////////
///////////////////////////////

void printMinLengthString(char s[]) {
  char stretchedString[30];
  sprintf(stretchedString, "%-25s", s);
  Serial.print(stretchedString);
}

void printDate() {
  char dateDisp[20];
  sprintf(dateDisp, "%02d/%02d/%04d", month(), day(), year());
  Serial.print(dateDisp);
}

void printTime() {
  char timeDisp[20];
  String amPm[] = {" AM", " PM"};
  sprintf(timeDisp, "%02d:%02d:%02d", hourFormat12(), minute(), second());
  Serial.print(timeDisp);
  Serial.print(amPm[isPM()]);
}

void printUptime() {
  char upTimeDisp[20];
  int uptimeSec = secsChrono.elapsed() % 60;
  int uptimeMin = (secsChrono.elapsed() / 60) % 60;
  int uptimeHour = (secsChrono.elapsed() / 60 / 60) % 60;
  int uptimeDay = (secsChrono.elapsed() / 60 / 60 / 24) % 24;
  sprintf(upTimeDisp, "%02d:%02d:%02d:%02d", uptimeDay, uptimeHour, uptimeMin, uptimeSec);
  Serial.print(upTimeDisp);
}
void printTempHum() {
  float t = sht31.readTemperature() * 1.8 + 32;
  float t2 = rtc.getTemp() * 1.8 + 32;
  if (!returnFarenheit) {
    t = sht31.readTemperature();
    t2 = rtc.getTemp();
  }
  float h = sht31.readHumidity();
  if (! isnan(t)) {  // check if 'is not a number'
    printMinLengthString("Temperature:"); Serial.print(t); Serial.print(" "); Serial.print(t2); 
  if (returnFarenheit) {
      Serial.println(" F");
    } else {
      Serial.println(" C");
    }
  } else {
    Serial.println("Failed to read temperature");
  }
  if (! isnan(h)) {  // check if 'is not a number'
    printMinLengthString("Humidity:"); Serial.print(h); Serial.println(" %");
  } else {
    Serial.println("Failed to read humidity");
  }
}


void toggleSolenoidOn(int x) {
  int minZero = max(x - 1, 0); //ensures >=0 array pointer
  //not already on? previous run finished? previous not running?
  if (!solenoidState[x] && solenoidChronoOn[x].hasPassed(solenoidOnTimer * 4 + solenoidOffTimer) && !solenoidState[minZero] && !anySolOn) {
    digitalWrite (solenoidPin[x], LOW);// write on
    anySolOn = !anySolOn; //toggle bool
    solenoidState[x] = !solenoidState[x];// toggle bool
    solenoidChronoOn[x].restart(); //reset solOn (solOn time = solenoidOnTimer*4 1000*4+solOff)
    //debug block
    //    Serial.print(x);
    //    Serial.println(" on.");
  }
}

void toggleSolenoidOff(int x) {
  //on? and long enough?
  if (solenoidState[x] && solenoidChronoOn[x].hasPassed(solenoidOnTimer)) {
    digitalWrite (solenoidPin[x], HIGH);// write off
    anySolOn = !anySolOn; //toggle bool
    solenoidState[x] = !solenoidState[x];// toggle bool

    //debug block
    //    Serial.print(x);
    //    Serial.print(" off at ");
    //    Serial.println(solenoidChronoOn[x].elapsed());
    //    Serial.println(solenoidChronoOff.elapsed());

    //reset solOff?
    if (x == 3) {
      solenoidChronoOff.restart();
    }
  }
}

void printLightPWM() {
  //  char dateDisp[20];
  //  sprintf(dateDisp, "%02d/%02d/%04d", month(), day(), year());
  char pwmDisp[20];
  //  sprintf(pwmDisp, "%-4d%-4d%-4d%-4d", analogRead(debugReadPin[0]), analogRead(debugReadPin[1]), analogRead(debugReadPin[2]), analogRead(debugReadPin[3]));
  Serial.print(pwmDisp);
  //  for (int i = 0; i < 4; i++) {
  //    Serial.print(analogRead(lightPin[i]);
  //  }
}
void setSolenoidTiming() { // remember trigger on change via rotary encoder!
  if (lightState) {
    solenoidOnTimer = solenoidOn;
    solenoidOffTimer = solenoidOff;
  }
  else {
    solenoidOnTimer = solenoidOnNight;
    solenoidOffTimer = solenoidOffNight;
  }
}
void setLightSchedule() { // remember trigger on change via rotary encoder!
  //sets time on/off
  tmLightStart.Hour = setLightStartHMS[0];
  tmLightStart.Minute = setLightStartHMS[1];
  tmLightStart.Second = setLightStartHMS[2];
  tmLightEnd.Hour = setLightEndHMS[0];
  tmLightEnd.Minute = setLightEndHMS[1];
  tmLightEnd.Second = setLightEndHMS[2];
}

void updateLightLoop() {
  todayT = elapsedSecsToday(now());  //change to RTC.get() when RTC immplemented
  breakTime(todayT, todayTm); //convert T to TM

  tLightStart = elapsedSecsToday(makeTime(tmLightStart)); //elapsedSecsToday returns secs since midnight
  tLightEnd = elapsedSecsToday(makeTime(tmLightEnd));

  //midnight check
  if (tLightStart > tLightEnd) {
    tLightEnd = tLightEnd % SECS_PER_DAY + SECS_PER_DAY;
  }
}

void setLight() {
  if (
    ((tLightStart <= todayT && tLightEnd >= todayT) // todayT (current time) is between lightstart and lightend
     || (tLightEnd > SECS_PER_DAY && (tLightEnd - SECS_PER_DAY) >= todayT)) // or if lightend passes midnight and after todayT (current time)
  ) { // on check: past time on, before time off
    //tLightEnd = 32
    if (lightState == 0) { // light isn't already on
      lightState = 1;
      //    digitalWrite(pin, LOW); //check for which state is required for relay ON, perhaps define it and use the var RELAY_ON instead
      Serial.println("                                                  LIGHTS ON");
    }
  }
  else {
    if (lightState == 1) {
      lightState = 0;
      //    digitalWrite(pin, HIGH);
      Serial.println("                                                  LIGHTS OUT");
    }
  }
}
//debugs/readouts
void printTimeReadout() {
  printMinLengthString("Date:");
  printDate();

  Serial.println();
  printMinLengthString("Time:");
  printTime();
  //    Serial.println(now());

  Serial.println();
  printMinLengthString("Uptime:");
  printUptime();
  Serial.println();
}

void displaySolenoid()    {
  printMinLengthString("Solenoid State:");
  for (int i; i < 4; i++) {
    //print solenoid state
    Serial.print(solenoidState[i]); Serial.print(" ");
  }
  Serial.println();

  printMinLengthString("SolenoidOff elapsed:"); Serial.println(solenoidChronoOff.elapsed());
  printChrono.restart();

  printMinLengthString("PWM States:"); printLightPWM();
  Serial.println();
}

void displayLight() {
  /*Debug block*/
  printMinLengthString("todayT:       "); Serial.println(todayT);

  printMinLengthString("tLightStart:  "); Serial.print(hour(tLightStart)); Serial.print(":"); Serial.print(minute(tLightStart)); Serial.print(":"); Serial.print(second(tLightStart));
  Serial.println();

  printMinLengthString("tLightEnd:    "); Serial.print(hour(tLightEnd)); Serial.print(":"); Serial.print(minute(tLightEnd)); Serial.print(":"); Serial.print(second(tLightEnd));
  Serial.println();

  printMinLengthString("lightState:   ");
  Serial.println(lightState);
}

//interrupt functions
void buttonISR() {
  //display shit here for configs
  //should toggle indicator for a setting that will be changed via control knob
}

//https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
//requires
void rotaryISR() {

}
