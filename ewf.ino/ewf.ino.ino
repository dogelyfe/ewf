

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

#include <ClickEncoder.h>

#include <URTouchCD.h>
#include <URTouch.h>

#include <memorysaver.h>
#include <UTFT.h>

#include <Chrono.h>
#include <LightChrono.h>

#include <Time.h>
#include <TimeLib.h>

/**********************************************************************/
// interrupt0 pin for control pot
int interruptButtonPin = 2;

Chrono printChrono;
Chrono secsChrono(Chrono::SECONDS);



//water
int solenoidPin[] = {4, 5, 6, 7};
bool solenoidState[] = {false, false, false, false};
bool anySolOn = false;

unsigned long solenoidOn = 2000; //on for millisecs
unsigned long solenoidOff = 5000; //off
Chrono solenoidChronoOff;
Chrono solenoidChronoOn[4];
//int solenoidJustRan[] = {false, false, false, false};

//pump
int pressurePin = 18;
int pumpPin = 8;

float pressureReading;
float pressurePsi = 90; //starts at 90 to avoid pump instant-on

int pressureLow = 80;
int pressureHigh = 125;

//light
int lightPin[4] = {9, 10, 11, 12};
int lightPWMPin[4] = {13, 14, 15, 16};


/**********************************************************************/
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
    pinMode(lightPin[i], OUTPUT);
  }
  for (int i; i < 4; i++) {
    pinMode(lightPWMPin[i], OUTPUT);
  }
}

//adjustTime(43200);


/**********************************************************************/
void loop() {
  /*** Status/Readings ***/
  if (printChrono.hasPassed(1000)) {
//    time-

    Serial.println();
    printMinLengthString("Date:");
    printDate();
    
    Serial.println();
    printMinLengthString("Time:");
    printTime();
    
    Serial.println();
    printMinLengthString("Uptime:");
    printUptime();
    Serial.println();

    printMinLengthString("Solenoid State:");
    for (int i; i < 4; i++) {
      //print solenoid state
      Serial.print(solenoidState[i]);
      Serial.print(" ");
    }
    Serial.println();
    //
    printMinLengthString("SolenoidOff elapsed:");
    Serial.println(solenoidChronoOff.elapsed());
    printChrono.restart();
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

  /*** LIGHTING ***/
  //PWM
  // arduino 5+ --> resistor (1k-10k 4.7K optimal?)(or 1k-100k --> transistor -->dimming dim-
  //battery --> voltage regulator (to transistor emitter) with caps for stability (recommend 20uF on output ) --> dim+
  //https://www.rollitup.org/t/meanwell-led-drivers-3-in-1-dimming-function.838760/page-2
  //may invert duty cycle
  //try without the voltage regulator from battery, since the driver may output its own 10v supply!

  //  analogWrite(lightPWMPin[i],

  /*** TEMPERATURE ***/
  //


  /*** HUMIDITY ***/


}

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
  if (!solenoidState[x] && solenoidChronoOn[x].hasPassed(solenoidOn * 4 + solenoidOff) && !solenoidState[minZero] && !anySolOn) {
    digitalWrite (solenoidPin[x], LOW);// write on
    anySolOn = !anySolOn; //toggle bool
    solenoidState[x] = !solenoidState[x];// toggle bool
    solenoidChronoOn[x].restart(); //reset solOn (solOn time = solenoidOn*4 1000*4+solOff)
    //debug block
    //    Serial.print(x);
    //    Serial.println(" on.");
  }
}

void toggleSolenoidOff(int x) {
  //on? and long enough?
  if (solenoidState[x] && solenoidChronoOn[x].hasPassed(solenoidOn)) {
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

//interrupt functions
void buttonISR() {
  //display shit here for configs
  //should toggle indicator for a setting that will be changed via control knob
}

//https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
//requires
void rotaryISR() {

}





