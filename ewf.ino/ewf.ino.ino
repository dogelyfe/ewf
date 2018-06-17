

//SETUP
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
#include <ClickEncoder.h>

#include <URTouchCD.h>
#include <URTouch.h>

#include <memorysaver.h>
#include <UTFT.h>

#include <Chrono.h>
#include <LightChrono.h>

#include <Time.h>
#include <TimeLib.h>

///////////////////////////////
/////////// INITS //////////////
///////////////////////////////

// interrupt0 pin for control pot
int interruptButtonPin = 2;

Chrono printChrono;
Chrono secsChrono(Chrono::SECONDS);


//water
int solenoidPin[] = {22, 23, 24, 25}; // [SET FOR MEGA]
bool solenoidState[] = {false, false, false, false};
bool anySolOn = false;

unsigned long solenoidOn = 2000; //on for millisecs, make adjustable by 1/10ths of sec
unsigned long solenoidOff = 5000; //off
unsigned long solenoidOnNight = 1000; //on for millisecs
unsigned long solenoidOffNight = 10000; //off
Chrono solenoidChronoOff, solenoidChronoOn[4];
unsigned long solenoidOnTimer, solenoidOffTimer;   //conditional holders for night/day value

//pump
int pressurePin = 18;
int pumpPin = 8;

float pressureReading;
float pressurePsi = 90; //starts at 90 to avoid pump instant-on

int pressureLow = 80;
int pressureHigh = 125;

//time
tmElements_t currentTm, todayTm, tmLightStart, tmLightEnd;
time_t currentT, todayT, tLightStart, tLightEnd;

//light
byte setLightStartHMS[3] = {00, 00, 10};
byte setLightEndHMS[3] = {00, 00, 15};
boolean lightState;
//bool lightState[] = {false, false, false, false};
//bool anyLit = false;

int lightPin[4] = {26, 27, 28, 29}; //light out pins (digital out) [SET FOR MEGA]
int lightPWMPin[4] = {9, 10, 11, 12}; //light pwm out pins (analog out) [SET FOR MEGA]

int lightIntensity = 90; //will receive value from pot adjust
int lightIntensityLast;

Chrono lightChrono;

//debug
int debugReadPin[4] = {97, 96, 95, 94};
int debugReading;


///////////////////////////////
/////////// SETUP /////////////
///////////////////////////////

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
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
  //solenoid relay triggers 22-25
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

  updateLightSchedule();
}

//adjustTime(43200);

///////////////////////////////
/////////// LOOP //////////////
///////////////////////////////

void loop() {
  /*** Status/Readings ***/
  if (printChrono.hasPassed(1000)) {
    Serial.println();
    displayTime();
    displaySolenoid();
    displayLight();
  }

  /*** WATERING CYCLE ***/
  //water loop 3.0

  if (lightState) {
    solenoidOnTimer = solenoidOn;
    solenoidOffTimer = solenoidOff;
  }
  else {
    solenoidOnTimer = solenoidOnNight;
    solenoidOffTimer = solenoidOffNight;
  }


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
  sprintf(pwmDisp, "%-4d%-4d%-4d%-4d", analogRead(debugReadPin[0]), analogRead(debugReadPin[1]), analogRead(debugReadPin[2]), analogRead(debugReadPin[3]));
  Serial.print(pwmDisp);
  //  for (int i = 0; i < 4; i++) {
  //    Serial.print(analogRead(lightPin[i]);
  //  }
}

void updateLightSchedule() { // remember trigger on change via rotary encoder!
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
void displayTime() {
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
    Serial.print(solenoidState[i]);
    Serial.print(" ");
  }
  Serial.println();

  printMinLengthString("SolenoidOff elapsed:");
  Serial.println(solenoidChronoOff.elapsed());
  printChrono.restart();

  printMinLengthString("PWM States:");
  printLightPWM();
  Serial.println();
}

void displayLight() {
  /*Debug block*/
  printMinLengthString("todayT:       ");
  Serial.println(todayT);

  printMinLengthString("tLightStart:  ");
  Serial.print(hour(tLightStart));
  Serial.print(":");
  Serial.print(minute(tLightStart));
  Serial.print(":");
  Serial.print(second(tLightStart));
  Serial.println();

  printMinLengthString("tLightEnd:    ");
  Serial.print(hour(tLightEnd));
  Serial.print(":");
  Serial.print(minute(tLightEnd));
  Serial.print(":");
  Serial.print(second(tLightEnd));
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





