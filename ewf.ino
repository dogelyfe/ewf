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
///// PINS /////
byte lightPin[4] = {26, 27, 28, 29}; //light relay pins (digital out) [SET FOR MEGA]
byte lightPWMPin[4] = {9, 10, 11, 12}; //light pwm out pins (analog out) [SET FOR MEGA]
byte solenoidPin[] = {22, 23, 24, 25}; // [SET FOR MEGA]
byte pressurePin = 18;
byte pumpPin = 8;
// byte rotaryData, rotaryClock;

// interrupt0 pin for control pot
byte interruptButtonPin = 2;
/////////////////////////////////
//////// END USER DEFINED ///////
/////////////////////////////////

//screen init
UTFT    myGLCD(CTE32_R2,38,39,40,41);
URTouch  myTouch( 6, 5, 4, 3, 2);
extern uint8_t SmallFont[];
// extern uint8_t arial_normal[];
int LCDpaddingX = 5, LCDpaddingY = 2;

//menu
  struct menuItem {
    String label;
    String (*printFn)();
    bool settable;
    bool hovered;
    bool selected;

    bool boolParam;
    int intParam;
    double doubleParam;
    
    // menuItem()
    //   //initialize list
    //   : settable(false), hovered(false), selected(false)
    //   {}
  };
  // menuArray[16]; 
  // String xyz;
  String (*ptrPrint)() = &printDate; //not sure why this is req-- throws multiple "func not declared in this scope" compile errors without
   // = printDate();
  menuItem menuArray[16] = {
    {"<DatePlaceholder>", printTime},
    {"Up   ", printUptime},
    {"Temp ", printTemp},
    {"Humi ", printHumidity},
    {"SolS ", printSolState},
    {"SOff ", printSolOff},
    {"PWMs ", printLightPWM},
    {"LStt ", printLightState},
    // {"Tody ", printTodayT},
    // {"", returnNull},
    {"", returnNull},
    {"LOn  ", printLightsOn},
    {"LOff ", printLightsOff},
    {"LStt ", printLightState}
  };

//RTC
DS3231  rtc(SDA, SCL);

Chrono printChrono;
Chrono secsChrono(Chrono::SECONDS);

//environment
Adafruit_SHT31 sht31 = Adafruit_SHT31();
//water
bool solenoidState[4] = {0, 0, 0, 0};
bool anySolOn = false;
Chrono solenoidChronoOff, solenoidChronoOn[4];
unsigned long solenoidOnTimer, solenoidOffTimer;   //conditional holders for night/day value
//pump
float pressureReading;
float pressurePsi = 90; //starts at 90 to avoid pump instant-on
//time
tmElements_t currentTm, todayTm, tmLightStart, tmLightEnd;
time_t currentT, todayT, tLightStart, tLightEnd;

//light
boolean lightState;
//bool lightState[] = {false, false, false, false};
//bool anyLit = false;

byte lightIntensity = 90; //will receive value from pot adjust, should cap at 100
byte lightIntensityLast;

Chrono lightChrono;

////debug
byte debugReadPin[4] = {97, 96, 95, 94};
//byte debugReading;


///////////////////////////////
/////////// SETUP /////////////
///////////////////////////////

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  //LCD init
  myGLCD.InitLCD();
  myGLCD.clrScr();

  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);

  myGLCD.setFont(SmallFont);
  // myGLCD.setFont(arial_normal);
  myGLCD.setBackColor(VGA_BLACK);
  
  //init menu w blue
  printLCDBG();


  rtc.begin();
  //uncomment to set
  // rtc.setDOW(TUESDAY);     // Set Day-of-Week to SUNDAY
  // rtc.setTime(15, 50, 0);     // Set the time to 12:00:00 (24hr format)
  // rtc.setDate(2, 7, 2018);   // day, month, year

  syncArdTime(); // copies rtc to ard now()

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
  if (printChrono.hasPassed(190)) {
    Serial.println();
    printLoop();
    // Serial.println(year());

    // if(menuArray[8].label == NULL){
    //     Serial.println("menuArray[8].label == NULL");
    //   }
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
      analogWrite(lightPWMPin[i], map(lightIntensity, 0, 100, 0, 255));
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

void printLoop() {
  // SERIAL BLOCK
  // Serial.print(printMinLengthString("Date:"));
  // Serial.print(printDate());

  // Serial.println();
  // Serial.print(printMinLengthString("Time:"));
  // Serial.print(printTime());
  // Serial.println();

  // Serial.print(printMinLengthString("Uptime:"));
  // Serial.print(printUptime());
  // Serial.println();

  // Serial.print(printMinLengthString("Temp:"));
  // Serial.print(printTemp());
  // Serial.println();

  // Serial.print(printMinLengthString("Humidity:"));
  // Serial.print(printHumidity());
  // Serial.println();

  // Serial.print(printMinLengthString("Sol States:"));
  // Serial.print(printSolState());
  // Serial.println();

  // Serial.print(printMinLengthString("Sol Off (s):"));
  // Serial.println(printSolOff());

  // Serial.print(printMinLengthString("PWM States:")); 
  // Serial.print(printLightPWM());
  // Serial.println();

  // Serial.print(printMinLengthString("Today (s):"));
  // Serial.println(printTodayT());

  // Serial.print(printMinLengthString("Lights On: ")); 
  // Serial.print(printLightsOn());
  // Serial.println();

  // Serial.print(printMinLengthString("Lights Off: ")); 
  // Serial.print(printLightsOff());
  // Serial.println();

  // Serial.print(printMinLengthString("lightState: "));
  // Serial.println(printLightState());

  printLCDMenu(); //LCD MENU

  printChrono.restart();
}

void printLCDMenu(){
  char LCDbuffer[50], LCDbuffer1[50];

  // menuArray[0].label = printDate();
  // menuArray[0].printFn = printTime;
  // menuArray[1].label = "Up   ";
  // menuArray[1].printFn = printUptime;
  // menuArray[2].label = "Temp ";
  // menuArray[2].printFn = printTemp;
  // menuArray[3].label = "Humi ";
  // menuArray[3].printFn = printHumidity;
  // menuArray[4].label = "SolS ";
  // menuArray[4].printFn = printSolState;
  // menuArray[5].label = "SOff ";
  // menuArray[5].printFn = printSolOff;
  // menuArray[6].label = "PWMs ";
  // menuArray[6].printFn = printLightPWM;
  // menuArray[7].label = "Tody ";
  // menuArray[7].printFn = printTodayT;
  // menuArray[8].label = "LOn  ";
  // menuArray[8].printFn = printLightsOn;
  // menuArray[9].label = "LOff ";
  // menuArray[9].printFn = printLightsOff;
  // menuArray[10].label = "LStt ";
  // menuArray[10].printFn = printLightState

  for (int i = 0; i <= 11; i++){ //set loop limit to highest array pointer
  //printlcd
    if (i == 0) {
      menuArray[0].label = printDate();
      myGLCD.setBackColor(VGA_BLUE);
    } else if(menuArray[i].label == NULL){
      myGLCD.setBackColor(VGA_BLUE);
    } else {
      myGLCD.setBackColor(VGA_BLACK);
    }
  // printMinLengthString(menuArray[i].label).toCharArray(LCDbuffer,50);
  (menuArray[i].label).toCharArray(LCDbuffer,50);
  menuArray[i].printFn().toCharArray(LCDbuffer1,50);
  strcat(LCDbuffer, LCDbuffer1);
  // sprintf(LCDbuffer, "%s%s", LCDbuffer, LCDbuffer1);
  myGLCD.print(LCDbuffer, LCDpaddingX, (LCDpaddingY + i * 13));
  // myGLCD.setBackColor(VGA_BLACK);
  }
}

String printMinLengthString(String s) {
  char stretchedString[30];
  s.toCharArray(stretchedString,30);
  sprintf(stretchedString, "%-13s", stretchedString);
  // Serial.print(stretchedString);
  return stretchedString;
}

String printDate() {
  char dateDisp[20];
  sprintf(dateDisp, "%02d/%02d/%04d  ", month(), day(), year());
  // Serial.print(dateDisp);
  String dateDispS = dateDisp;
  return dateDispS;
}

String printTime() {
  char timeDisp[20], amPm[4]; 
  if(isPM()){
    strcpy(amPm, " PM");
  } else {
    strcpy(amPm, " AM");
  }
  sprintf(timeDisp, "%02d:%02d:%02d", hourFormat12(), minute(), second());
  strcat(timeDisp, amPm);
  // Serial.print(timeDisp);
  return timeDisp;
}

String printUptime() {
  char upTimeDisp[20];
  int uptimeSec = secsChrono.elapsed() % 60;
  int uptimeMin = (secsChrono.elapsed() / 60) % 60;
  int uptimeHour = (secsChrono.elapsed() / 60 / 60) % 60;
  int uptimeDay = (secsChrono.elapsed() / 60 / 60 / 24) % 24;
  sprintf(upTimeDisp, "%02d:%02d:%02d:%02d", uptimeDay, uptimeHour, uptimeMin, uptimeSec);
  // Serial.print(upTimeDisp);
  return upTimeDisp;
}

String printTemp() {
  float t = sht31.readTemperature() * 1.8 + 32;
  float t2 = rtc.getTemp() * 1.8 + 32;
  String tempDisp;
  char FC[4];
  if (!returnFarenheit) {
    t = sht31.readTemperature();
    t2 = rtc.getTemp();
  }
  if (! isnan(t)) {  // check if 'is not a number'
    // Serial.print(t); Serial.print(" "); Serial.print(t2); 
    if (returnFarenheit) {
      strcpy(FC, "F");
      } else {
      strcpy(FC, "C");
      }
      // dtostrf(t, -6, 2, tOut); dtostrf(t2, -6, 2, t2Out);
      //printf(tempDisp, "%6.2f %s %s", t, t2Out, FC); //dont use s for FC
      tempDisp.concat(t); tempDisp.concat(", ");
      tempDisp.concat(t2); tempDisp.concat(" "); tempDisp.concat(FC);
      // Serial.print(tempDisp);
      return tempDisp;
  } else {
    // Serial.println("Failed to read temperature");
    return "Failed to read temperature";
  }
}
String printHumidity() {
  float h = sht31.readHumidity();
  String humDisp;
  if (! isnan(h)) {  // check if 'is not a number'
    //dtostrf(h, 6, 2, hOut);
    // strcpy(hOut, "%");
    // printf(humDisp, "%-6s %-2s", h, hOut);
    humDisp.concat(h); humDisp.concat(" %");
    // Serial.print(humDisp);
    return humDisp;
  } else {
    // Serial.println("Failed to read humidity");
    return "Failed to read humidity";
  }
}
String printSolState(){
  String solDisp;
  for (int i; i < 4; i++) {
    //print solenoid state
    if (solenoidState[i]){
      solDisp.concat("on  ");
    } else {
      solDisp.concat("off ");
    }
  }
  // Serial.print(solDisp);
  return solDisp;
}
String printSolOff(){
  char buffer[10];
  ltoa(solenoidChronoOff.elapsed(), buffer, 10);
  // solenoidChronoOff.elapsed().toCharArray(buffer, 10);
  return buffer;
}
String printTodayT(){
  char buffer[10];
  ltoa(todayT, buffer, 10);
  // solenoidChronoOff.elapsed().toCharArray(buffer, 10);
  return buffer; 
}
String printLightState(){
  if (lightState){
    return "on";
  } else {
    return "off";
  }
}
void syncArdTime(){
  Time holderTime = rtc.getTime();
  setTime(holderTime.hour, holderTime.min, holderTime.sec, holderTime.date, holderTime.mon, holderTime.year); 
  // setDate(uint8_t date, uint8_t mon, uint16_t year);
  currentT = now();
}
String returnNull(){
  return "";
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

String printLightPWM() {
  //  char dateDisp[20];
  //  sprintf(dateDisp, "%02d/%02d/%04d", month(), day(), year());
  String pwmDisp;
  char buffer[5];
  //  sprintf(pwmDisp, "%-4d%-4d%-4d%-4d", analogRead(debugReadPin[0]), 
  // analogRead(debugReadPin[1]), analogRead(debugReadPin[2]), analogRead(debugReadPin[3]));
  //  Serial.print(pwmDisp);
  //  for (int i = 0; i < 4; i++) {
  //    Serial.print(analogRead(lightPin[i]);
  //  }
  for (int i; i < 4; i++){
    sprintf(buffer, "%-4d", analogRead(debugReadPin[i]));
    pwmDisp.concat(buffer);
  }
  // Serial.print(pwmDisp);
  return pwmDisp;
}

String printLightsOn(){
  char buffer[20];
  sprintf(buffer, "%02d:%02d:%02d", hour(tLightStart), minute(tLightStart), second(tLightStart));

  // lightsOnDisp.concat(hour(tLightStart)); lightsOnDisp.concat(":"); 
  // lightsOnDisp.concat(minute(tLightStart)); lightsOnDisp.concat(":"); 
  // lightsOnDisp.concat(second(tLightStart));
  String lightsOnDisp = String(buffer);
  if (isAM(tLightStart)){
    lightsOnDisp.concat(" AM");
  } else {
    lightsOnDisp.concat(" PM");
  }
  // Serial.print(lightsOnDisp);
  return lightsOnDisp;
}
String printLightsOff(){
  char buffer[20];
  sprintf(buffer, "%02d:%02d:%02d", hour(tLightEnd), minute(tLightEnd), second(tLightEnd));
  // lightsOffDisp.concat(hour(tLightEnd)); lightsOffDisp.concat(":"); 
  // lightsOffDisp.concat(minute(tLightEnd)); lightsOffDisp.concat(":"); 
  // lightsOffDisp.concat(second(tLightEnd));
  String lightsOffDisp = String(buffer);
  if (isAM(tLightEnd)){
    lightsOffDisp.concat(" AM");
  } else {
    lightsOffDisp.concat(" PM");
  }
  // Serial.print(lightsOffDisp);
  return lightsOffDisp;
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
     || (tLightEnd > SECS_PER_DAY && (tLightEnd - SECS_PER_DAY) >= todayT)) // or lightend passes midnight and after todayT (current time)
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

void printLCDBG(){
  myGLCD.setColor(VGA_BLUE);
  myGLCD.fillRect(0, 0, 200, 14);
  myGLCD.fillRect(0, 105, 200, 118);
  myGLCD.setColor(20,20,20);
  myGLCD.fillRect(200, 0, 319, 240);
  myGLCD.setColor(VGA_WHITE);
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
