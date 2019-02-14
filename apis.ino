/* -------------------------------------
 Automatic Plant Irrigation System - APIS
   Code Version 1.8.5
   Parameters Version 13

 Change Log:
  2015-02-27:
    v0.9.0 - added support for RTC for no irrigation at night & watering log & brightness control
    v0.9.0 - fixed run log issue with index overwrite
  2015-03-10:
    v1.0.0 - support for parameterized weekend adjustment time
  2015-03-24:
    v1.1.0 - support for adjusting settings and time with 3 buttons: Select, + and -
    v1.1.0 - added suturation in minutes to the list of configurable parameters
    v1.1.0 - RTC_Millis is used for testing
  2015-04-27:
    v1.2.0 - delay after exiting the log (bug fix) - v11 triggered interrupt too quickly
    v1.2.0 - force pumping as one of the options via buttons. Only works if hum < upper threshold. 
    v1.2.0 - different handling of error situation (Err for 24h and then restart to try again)
    v1.2.0 - compiled with PROGMEM support in TM1650 library
  2015-05-16:
    v1.4.0 - hardware change: power soil probe from the digital pin to limit the time probe is under voltage
    v1.4.0 - change measure routine to enable power digital pin first/wait for 1 minute to settle and then measure
    v1.4.0 - move to measuring once per 1 hour with display off most of the time
    v1.4.0 - display humidity only once per hour -OR- on a button press on demand - no need to show previous run since a log option is available
  2015-05-18:
    v1.5.0 - use of PinChange library to enable interrupts on all button pins
  2015-08-25:
    v1.6.0 - fix bug: saturation animation not turned off for the night
    v1.6.0 - enable probe during water runs, and disable during saturation
    v1.6.0 - switched to EnableInterrupt library
    v1.6.0 - switched to TaskScheduler 1.8.0
    v1.6.0 - implement automatic DST switchover via use of Timezone library. Device's RTC will run on UTC time. 
  2015-10-21:
    v1.7.0 - switch to EEPROM.update - a write function only writes value if it is different from the value already stored in the cell
  2015-11-16:
    v1.7.0 - switch to DirectIO library
    v1.7.0 - implement button repeat with varying (increasing) rate
  2015-11-17:
    v1.7.1 - debounce the buttons with 10 ms delay
    v1.7.1 - complie against the fork of Time to exclude time provider sync at every call to now()
  2015-11-19
    v1.8.0 - more extensive logging, including start/stop time of the watering run, number of runs and start/stop humidity
  2015-11-20
    v1.8.1 - bug fix: Settings timeout should not kick in while displying running text
    v1.8.1 - bug fix: Cancel settings leads to 30 seconds delay instead of 5 seconds (TaskScheduler bug discovered)
  2015-12-23
    v1.8.2 - complied with latest TM1650 ro enable gradual brightness 7 seg panel On and Off
    v1.8.2 - compiled with the latest TaskScheduler v2.0.0
  2016-02-18
    v1.8.3 - compiled with the latest TaskScheduler v2.1.0
    v1.8.3.- updates to the error handling (device is hard reset after 24 hours)
  2016-03-07
    v1.8.4 - added a pump reverse at the end of the run to drain the tubes and prevent mold from forming inside them
  2016-11-09
    v1.8.5 - explicitly added '#include <TimeLib.h>' to compile on 1.6.12 without errors
    v1.8.5 - added functionality to detect water leak under the pot and stop watering if detected (or not even start watering if water is still under the pot)
  2016-11-17
    v1.8.5 - added water leak indication (all dots illuminated) when there is water leaking from below the pot
    
 ----------------------------------------*/

#include <TM1650.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>
#include <AvgFilter.h> 
#include <avr/wdt.h>

//#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>

#include <TimeLib.h>  // v1.8.5 - just including Time.h does not work anymore for now() method (for some reason)
#include <Timezone.h> 

#include <DirectIO.h>

//#define _DEBUG_
#define _TEST_
#define _USERTCMILLIS_


#define RETRIES 3
#define RETRIES_MIN 1
#define RETRIES_MAX 10

// Current pump is rated at 100 ml/min
// split BTW 2 plants: 50 ml per run
// max watering is 200 ml with 4 default runs

// Time to run pump within one water run
#define WATERTIME 60 //Seconds
#define WATERTIME_MIN 5 //Seconds
#define WATERTIME_MAX 120 //Seconds

#define WATERTIME_REV 10 // Seconds

// Time to saturate
#define SATURATE 5 // 5 minutes
#define SATURATE_MIN 1 // 1 minute
#define SATURATE_MAX 90 // 10 minutes

// % soil humidity to start pumping (low threshold)
#define NEEDWATER 50 // % to start pumping
#define NEEDWATER_MIN 20 // % to start pumping
#define NEEDWATER_MAX 75 // % to start pumping

// % soil humidity to stop pumping (high threshold)
#define STOPWATER 60 // % to stop pumping
#define STOPWATER_MIN 25 // % to stop pumping
#define STOPWATER_MAX 90 // % to stop pumping

// Hour of the day to "go to sleep" (i.e., do not operate after this hour)
#define GOTOSLEEP 22 // hour to go to sleep
#define GOTOSLEEP_MIN  0 // hour to go to sleep
#define GOTOSLEEP_MAX 24 // hour to go to sleep

// Hour of the day to "wake up" (i.e., operate after this hour)
#define WAKEUP    7 // hour to wake up
#define WAKEUP_MIN  0 // hour to wake up
#define WAKEUP_MAX 24 // hour to wake up

// Number of hours to add to wake up time on a weekend
#define WEEKENDADJ  2  // number of hours to add for the wakeup on a weekend
#define WEEKENDADJ_MIN  0  // number of hours to add for the wakeup on a weekend
#define WEEKENDADJ_MAX  12  // number of hours to add for the wakeup on a weekend

#define PARAMADDR   128
#define LOGIDXADDR  (PARAMADDR+sizeof(parameters))
#define LOGADDR     (LOGIDXADDR+2)
#define MAXLOGS     20
#define RUNNING_DISPLAY_START_DELAY  1000
#define RUNNING_DISPLAY_DELAY  250
#define SENSOR_OUT_VALUE  1000

#define MOISTUREPIN   A1
#define LEAKPINPWR    A2  // ground or power pin for under the pot moisture measurement
#define LEAKPINMSR    A3  // input_pullup pin for under the pot moisture measurement
#define POWERUP_PIN   10
#define BTN_SEL_PIN   2
#define BTN_PLUS_PIN  8
#define BTN_MINUS_PIN 9
#define BTN_INIT      2000
#define BTN_REPEAT    1000
#define BNT_RPT_CNT   5
#define BTN_RAPID     250
#define BNT_DEBOUNCE  20

AnalogInput<MOISTUREPIN>  pMoisture;
Output<POWERUP_PIN>       pPowerup;
Input<BTN_SEL_PIN>        pSelect;
Input<BTN_PLUS_PIN>       pPlus;
Input<BTN_MINUS_PIN>      pMinus;
Output<13>                pLED;

// L293D pinout
#define M1P1 6
#define M1P2 7
#define M1E1 5

Output<M1P1>              pM1P1;
Output<M1P2>              pM1P2;
Output<M1E1>              pM1E1;


const char CToken[] = "APIS13\0"; // Eeprom token: Automatic Plant Irrigation System

int state;
boolean error, night_time;

#ifndef _TEST_
int   currentHumidity = 0;
#else
int   currentHumidity = 60;
#endif

enum Display_Options : byte {
  DHUMIDITY,
  DHIGH_MARK,
  DLOW_MARK,
  DWATER_TIME,
  DWATER_RETRIES,
  DERROR,
  DGOODNIGHT,
  DLOG,
  DLOG_V,
  DLOG_P,
  DSTORED,
  DCANCELLED,
  DCLK_SET,
  DSET,
  DSET_UL, 
  DSET_LL, 
  DSET_RT, 
  DSET_PT,
  DSET_ST, 
  DSET_SL, 
  DSET_UP,  
  DSET_AD, 
  DCLK,
  DCLK_CN, DCLK_YR, DCLK_MT, DCLK_DY, DCLK_HR, DCLK_MN,
  DFRN,
  DTIME,
  DRUN,
  DEMPTY,
};

Display_Options displayNow = DHUMIDITY;
TM1650 panel;
bool   panel_status;
byte   panel_brightness = TM1650_MAX_BRIGHT;
// volatile int buttonPressed;

#ifndef _TEST_
#define TMEASURE_INTERVAL    3570  // 59 minutes
#define TMEASURE_PRIME       30 // then 1 minute to power up the probe = 1 hour sensor interval
#else
#define TMEASURE_INTERVAL    45 
#define TMEASURE_PRIME       5
#endif

#define TMEASURE_WATERTIME   5  // measure every 5 seconds while watering

#define TDISPLAY_INTERVAL    3600
#define TDISPLAY_TIMEOUT     30
#define TDISPLAY_SHORT_DELAY 5
#define TGOODNIGHT_INTERVAL  60
#define TSETTOUT_INTERVAL    15
#define ANIMATION_MILLIS     250
#define TI_INTERVAL          500 //button press delay 

Scheduler ts;

// Forward definition of all callback methods is now required by v1.6.6
void buttonISR();
void goodnightCallbackInit();
void measureCallback();
void settingsToutCallback();
void displayCallback();
void interruptCallback();
void waterCallback();
bool waterOnEnable();
void waterOnDisable();
void animationWaterCallback();
void errorCallback();
void timeSyncCallback();
void displayRunningCallback();
void displayTimeout();
void waterOffCallback();
void motorOff();
bool motorReverse();

Task tIsr       (BTN_REPEAT, TASK_FOREVER, &buttonISR, &ts);
Task tGoodnight (TGOODNIGHT_INTERVAL * TASK_SECOND, TASK_FOREVER, &goodnightCallbackInit, &ts, true);
Task tMeasure   (TMEASURE_PRIME * TASK_SECOND, TASK_FOREVER, &measureCallback, &ts, true);
Task tSettingsTimeout (TSETTOUT_INTERVAL * TASK_SECOND, TASK_ONCE, &settingsToutCallback, &ts);
Task tDisplay   (TDISPLAY_INTERVAL * TASK_SECOND, TASK_FOREVER, &displayCallback, &ts, true);
Task tDisplayRunning (RUNNING_DISPLAY_START_DELAY, TASK_FOREVER, &displayRunningCallback, &ts);
Task tDisplayTimeout (TDISPLAY_TIMEOUT * TASK_SECOND, TASK_FOREVER, &displayTimeout, &ts, true); 
Task tInterrupt (TI_INTERVAL, TASK_ONCE, &interruptCallback, &ts);
Task tWater     (SATURATE * TASK_MINUTE, RETRIES, &waterCallback, &ts, false, &waterOnEnable, &waterOnDisable);
Task tWaterOff  (WATERTIME * TASK_SECOND, TASK_ONCE, &waterOffCallback, &ts);
Task tWaterReverse(WATERTIME_REV * TASK_SECOND * 3, TASK_ONCE, NULL, &ts, false, &motorReverse, &motorOff);
Task tWaterAnimation (ANIMATION_MILLIS, TASK_FOREVER, &animationWaterCallback, &ts);
Task tError     (TASK_IMMEDIATE, TASK_FOREVER, &errorCallback, &ts);
#ifdef _TEST_
void testHumidityCallback();
Task tTestHumidity(20000, TASK_FOREVER, &testHumidityCallback, &ts);  // temporarly call now() every minute to advance system time in Time library
#endif

struct {
  char      token[7];   //  +0:  6 digit token = APISxx, where xx is a version + '\0'
  byte      high;       //  +7:  high humidity mark - stop watering
  byte      low;        //  +8:  low humidity mark - start watering
  byte      retries;    //  +9:  number of watering runs before give up (if high not reached)
  byte      watertime;  //  +10: pumping duration
  byte      saturate;   //  +11: saturation duration
  byte      gotosleep;  //  +12: hour to go goodnight (e.g., 22)
  byte      wakeup;     //  +13: hour to wake up (e.g., 08)
  byte      wkendadj;   //  +14: weekend wake up adjustment time
} parameters;

struct {
  time_t    water_start;  // time watering run started
  byte      hum_start;    // humidity at the start
  byte      num_runs;     // number of runs it took to water
  byte      run_duration; // run durations
  time_t    water_end;    // time watering stopped
  byte      hum_end;      // humidity at the end of run
} water_log;

struct {
  byte  cn;
  byte  yr;
  byte  mt;
  byte  dy;
  byte  hr;
  byte  mn;
} mytime;

byte  *adjParam;
int    paramIndex;

#define PARAMIDXSTART  0
#define CLOCKIDXSTART  8
#define DAYSIDX        11

byte paramMin[] = {STOPWATER_MIN, NEEDWATER_MIN, RETRIES_MIN, WATERTIME_MIN, SATURATE_MIN, GOTOSLEEP_MIN, WAKEUP_MIN, WEEKENDADJ_MIN, 19, 00, 01, 01, 00, 00 };
byte paramMax[] = {STOPWATER_MAX, NEEDWATER_MAX, RETRIES_MAX, WATERTIME_MAX, SATURATE_MAX, GOTOSLEEP_MAX, WAKEUP_MAX, WEEKENDADJ_MAX, 29, 99, 12, 31, 23, 59 };
char paramChar[] = "ULrPSbuacytdhn";

#ifndef _USERTCMILLIS_
RTC_DS1307 rtc;
#else
RTC_Millis rtc;
#endif

// Support for timezones:
//US Eastern Time Zone (New York)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);


byte  logIndex;
byte  logCount;

//
// CODE
//
void motorOff()
{
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": motorOff"));
#endif
//  digitalWrite(M1E1, LOW);
//  digitalWrite(M1P1, LOW);
//  digitalWrite(M1P2, LOW);
  pM1E1 = LOW;
  pM1P1 = LOW;
  pM1P2 = LOW;
}

void motorOn()
{
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": motorOn"));
#endif
//  digitalWrite(M1P1, HIGH);
//  digitalWrite(M1P2, LOW);
//  digitalWrite(M1E1, HIGH);

  pM1P1 = HIGH;
  pM1P2 = LOW;
  pM1E1 = HIGH;
}

bool motorReverse()
{
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": motorOn"));
#endif
//  digitalWrite(M1P1, HIGH);
//  digitalWrite(M1P2, LOW);
//  digitalWrite(M1E1, HIGH);

  pM1P1 = LOW;
  pM1P2 = HIGH;
  pM1E1 = HIGH;

  return true;
}

void panelOn() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": panelOn"));
#endif

  panel_status = true;
  panel.displayOn();
  panel.setBrightnessGradually(panel_brightness);
}

void panelOff() {
  #ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": panelOff"));
#endif

  panel_status = false;
  panel.setBrightnessGradually(TM1650_MIN_BRIGHT);
  panel.displayOff();
}


void measurePower(bool aStatus) {
#ifdef _TEST_
//  digitalWrite(13, aStatus ? HIGH : LOW);
  pLED = aStatus ? HIGH : LOW;
#else
//  digitalWrite(POWERUP_PIN, aStatus ? HIGH : LOW);
  pPowerup = aStatus ? HIGH : LOW;
#endif
}

bool isLeak() {
  bool r;
  pinMode(LEAKPINMSR,INPUT_PULLUP);
  r = digitalRead(LEAKPINMSR) == LOW;
  pinMode(LEAKPINMSR,INPUT); 
  return r; 
}


long  humData[5];
avgFilter hum(5, humData);

long measureHumidity()
{
  long r, l;
  
//  l = analogRead( MOISTUREPIN );
  l = pMoisture;

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.print(F(": measureHumidity. l = "));
  Serial.println(l);
#endif

#ifdef _TEST_
  return currentHumidity;
#endif


  if (l > SENSOR_OUT_VALUE) return 0;
//  if (l > SENSOR_OUT_VALUE) return hum.value(currentHumidity);
  
  r = (1023 - l) * 100L / 1023L;
  return hum.value(r);
}




#ifdef _TEST_
void testHumidityCallback() {
  
  if (currentHumidity == 0) currentHumidity = 65;
  if (tWater.isEnabled()) {
    currentHumidity++;
    if (tWaterOff.isEnabled())
      tTestHumidity.setInterval(2000);
    else
      tTestHumidity.setInterval(10000);    
  }
  else {
    currentHumidity--;
    tTestHumidity.setInterval(10000);
  }
  
  if (currentHumidity > 95) currentHumidity = 95;
  if (currentHumidity < 20) currentHumidity = 20;
  
}
#endif



void loadParameters() {

  int paramLen = sizeof (parameters);
  byte *p;

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": loadParameters."));
#endif
  // Let's see if we have the defaults stored already
  // First lets read the token.

  p = (byte *) &parameters;
  for (int i = 0; i < paramLen; i++, p++) {
    *p = EEPROM.read(PARAMADDR + i);
  }
  logIndex = EEPROM.read(LOGIDXADDR);
  logCount = EEPROM.read(LOGIDXADDR + 1);
  if (strcmp(CToken, parameters.token) != 0) {
    // Write down token and defaults
    strncpy(parameters.token, CToken, 7);
    parameters.high = (byte) STOPWATER;
    parameters.low = (byte) NEEDWATER;
    parameters.retries = (byte) RETRIES;
    parameters.watertime = (byte) WATERTIME;
    parameters.saturate = (byte) SATURATE;
    parameters.gotosleep = (byte) GOTOSLEEP;
    parameters.wakeup = (byte) WAKEUP;
    parameters.wkendadj = (byte) WEEKENDADJ;
    saveParameters();
    logIndex = 0;
    logCount = 0;
    EEPROM.update(LOGIDXADDR, logIndex);
    EEPROM.update(LOGIDXADDR + 1, logCount);
  }
}


void saveParameters() {
  int paramLen = sizeof (parameters);
  byte *p;

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": saveParameters."));
#endif

  p = (byte *) &parameters;
  for (int i = 0; i < paramLen; i++, p++) {
    EEPROM.update (PARAMADDR + i, *p);
  }
}



void writeLogEntry() {

  byte *p = (byte *) &water_log;
  int   len = sizeof(water_log);
  
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": writeLogEntry."));
#endif

  for (int i = 0; i < len; i++, p++) {
    EEPROM.update (LOGADDR + logIndex * len + i, *p);
  }
  if (++logIndex >= MAXLOGS) logIndex = 0;
  if (++logCount >= MAXLOGS) logCount = MAXLOGS;
  EEPROM.update(LOGIDXADDR, logIndex);
  EEPROM.update(LOGIDXADDR + 1, logCount);
}


bool lastLogEntry() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": lastLogEntryDate."));
#endif
  
  return readLogEntry(0);
}


bool readLogEntry(int aIndex) {
  byte *p = (byte *) &water_log;
  int   len = sizeof(water_log);

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": readLogEntryDate."));
#endif

  if (logCount == 0) return false;
  if (aIndex < 0 || aIndex > logCount) return false;
  int index = logIndex - aIndex - 1;
  if (index < 0) index += MAXLOGS;
  for (int j = 0; j < len; j++, p++) {
    *p = EEPROM.read (LOGADDR + index * len + j);
  }
  return true;
}



void measurePowerupCallback() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": measurePowerupCallback."));
#endif
  measurePower(true);
  tMeasure.set(TMEASURE_PRIME * TASK_SECOND, TASK_FOREVER, &measureCallback);
  tMeasure.enableDelayed();
}


void measureCallback() {

#ifdef _TEST_
  tTestHumidity.enableIfNot();
#endif

  for (int i=0; i<5; i++) {
    currentHumidity = measureHumidity();
    delay(100);
  }

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.print(F(": measureCallback. Humidity ="));
  Serial.print(currentHumidity);
  Serial.println("%");
#endif

  if (!isLeak() && currentHumidity > 0 && currentHumidity < parameters.low) {
    if (!tWater.isEnabled()) {
      showHumidity(0);
      tWater.setInterval( parameters.saturate * TASK_MINUTE );
      tWater.setIterations( parameters.retries + 1 );
      tWater.restart();
      tMeasure.set(TMEASURE_WATERTIME * TASK_SECOND, TASK_FOREVER, &measureCallback);
      tMeasure.enableDelayed();  
      error = false;
    }
    return;
  }
  
  showHumidity(1);
  if ( (tWater.isEnabled() && currentHumidity >= parameters.high) || isLeak()) {
    tWater.disable();
  }
  
  if (!tWater.isEnabled() ) {
    measurePower(false);
    tMeasure.set(TMEASURE_INTERVAL * TASK_SECOND, TASK_FOREVER, &measurePowerupCallback);
    tMeasure.enableDelayed();  
  }
}

bool waterOnEnable() {
  
  if ( night_time || isLeak() ) {
    motorOff();
    return false;
  } 

  water_log.water_start = now();
  water_log.hum_start = currentHumidity;
  water_log.num_runs = 0;
  water_log.run_duration = parameters.watertime;

  return true;
}

void waterOnDisable() {
  
  tWaterOff.disable();
  tWaterAnimation.disable();
  motorOff();
  
  water_log.water_end = now();
  water_log.hum_end = currentHumidity;  
  
  writeLogEntry();  
  tWaterReverse.restartDelayed();  // reverse the pump to drain the tubes
}

void waterCallback() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.print(F(": waterCallback. Iteration: "));
  Serial.println(tWater.getIterations());
#endif

  if (!tWater.isLastIteration()) {
    motorOn();
    water_log.num_runs++;
    if ( tWater.isFirstIteration() ) {
      tWaterOff.set((parameters.watertime + WATERTIME_REV) * TASK_SECOND, 1, &waterOffCallback);  // add extra time to prime the tubes
    }
    else {
      tWaterOff.set(parameters.watertime * TASK_SECOND, 1, &waterOffCallback);
    }
    tWaterOff.restartDelayed();
    tWaterAnimation.setCallback(&animationWaterCallback);
    tWaterAnimation.enable();
    showHumidity(0);
    return;
  }
  currentHumidity = measureHumidity();
  if ( currentHumidity >= parameters.low ) {
    tWater.disable();
    showHumidity(1);
    return;
  }
  motorOff;
  measurePower(false);
  ts.disableAll();
  tError.enable();
}

void waterOffCallback() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": waterOffCallback."));
#endif
  motorOff();
  tWater.delay();
  tWaterAnimation.setCallback(&animationSaturateCallback);
  tWaterAnimation.enable();
}


int animCounter ;

void animationWaterCallback() {
  panel.setDot(animCounter++, false);
  animCounter &= 3;
  panel.setDot(animCounter, true);
}

void animationSaturateCallback() {
  switch (animCounter) {
    case 0:
      panel.setDot(0, false);
      panel.setDot(1, true);
      panel.setDot(2, true);
      panel.setDot(3, false);
      break;

    case 1:
      panel.setDot(0, true);
      panel.setDot(1, false);
      panel.setDot(2, false);
      panel.setDot(3, true);
      break;

    case 2:
    case 3:
      panel.setDot(0, false);
      panel.setDot(1, false);
      panel.setDot(2, false);
      panel.setDot(3, false);
      break;
  }
  animCounter++;
  animCounter &= 3;
}



void displayTimeout() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": displayTimeout."));
#endif  
  if (!tWater.isEnabled()) panelOff();
}


#define LINELEN (100)
void displayCallback() {
  char line[LINELEN+1];
  int h = currentHumidity;
  time_t  tnow;

  tDisplayTimeout.delay();
  
  if (tDisplayRunning.isEnabled()) return;

  switch (displayNow) {
    case DHUMIDITY:
      snprintf(line, 5, "h%3d", h);
      break;

    case DGOODNIGHT:
      snprintf(line, LINELEN, "good night    ");
      line[strlen(line) - 1] |= 0x80; // put a dot so we see it's on
      panel.displayRunning(line); 
      tDisplayRunning.setInterval(RUNNING_DISPLAY_START_DELAY);
      tDisplayRunning.enableDelayed();
      switchDisplayNow(DEMPTY, 1);
      break;

    case DEMPTY:
      sprintf(line, "    ");
      line[3] |= 0x80;
      break;
      
    case DSTORED:
      sprintf(line, "Strd");
      showHumidity(TDISPLAY_SHORT_DELAY * TASK_SECOND);
      break;

    case DCANCELLED:
      sprintf(line, "Canc");
      showHumidity(TDISPLAY_SHORT_DELAY * TASK_SECOND);
      break;

    case DCLK_SET:
      sprintf(line, "Set ");
      showHumidity(TDISPLAY_SHORT_DELAY * TASK_SECOND);
      break;

    case DSET:
      sprintf(line, "SETT");
      break;

    case DCLK:
      sprintf(line, "CLOC");
      break;

    case DFRN:
      sprintf(line, "Frun");
      break;

    case DLOG:
      sprintf(line, "LOG ");
      break;

    case DSET_UL:
    case DSET_LL:
    case DSET_PT:
    case DSET_RT:
    case DSET_ST:
    case DSET_SL:
    case DSET_UP:
    case DSET_AD:
    case DCLK_CN:
    case DCLK_YR:
    case DCLK_MT:
    case DCLK_DY:
    case DCLK_HR:
    case DCLK_MN:
      snprintf(line, 5, "%4d", *adjParam);
      line[0] = paramChar[paramIndex] | 0x80;
      break;

    case DLOG_V:
        snprintf(line, LINELEN, "%02d start on %02d-%02d at %02d%02d %02d h %02d runs end on %02d-%02d at %02d%02d %02d h", paramIndex+1,\
          month(myTZ.toLocal(water_log.water_start)), day(myTZ.toLocal(water_log.water_start)), hour(myTZ.toLocal(water_log.water_start)),\
          minute(myTZ.toLocal(water_log.water_start)), water_log.hum_start, water_log.num_runs,\
          month(myTZ.toLocal(water_log.water_end)), day(myTZ.toLocal(water_log.water_end)), hour(myTZ.toLocal(water_log.water_end)),\
          minute(myTZ.toLocal(water_log.water_end)), water_log.hum_end );
        line[1] |= 0x80;        
        line[22] |= 0x80; //enable dot
        line[56] |= 0x80; //enable dot

      panel.displayRunning(line); 
      tDisplayRunning.setInterval(RUNNING_DISPLAY_START_DELAY);
      tDisplayRunning.enableDelayed();

      switchDisplayNow(DLOG_P, 1);
      return;

    case DTIME:
      tnow = myTZ.toLocal( now() );
      snprintf(line, LINELEN, "date %04d-%02d-%02d %02d%02d", year(tnow), month(tnow), day(tnow), hour(tnow), minute(tnow));
      line[17] |= 0b10000000;  //dot
      panel.displayRunning(line); 
      tDisplayRunning.setInterval(RUNNING_DISPLAY_START_DELAY);
      tDisplayRunning.enableDelayed();
     
      showHumidity(TDISPLAY_SHORT_DELAY * TASK_SECOND);
      return;

    case DRUN:
      if (lastLogEntry()) {
        snprintf(line, LINELEN, "last run start on %02d-%02d at %02d%02d %02d h %02d runs end on %02d-%02d at %02d%02d %02d h",\
          month(myTZ.toLocal(water_log.water_start)), day(myTZ.toLocal(water_log.water_start)), hour(myTZ.toLocal(water_log.water_start)),\
          minute(myTZ.toLocal(water_log.water_start)), water_log.hum_start, water_log.num_runs,\
          month(myTZ.toLocal(water_log.water_end)), day(myTZ.toLocal(water_log.water_end)), hour(myTZ.toLocal(water_log.water_end)),\
          minute(myTZ.toLocal(water_log.water_end)), water_log.hum_end );
        line[28] |= 0x80; //enable dot
        line[62] |= 0x80; //enable dot
      }
      else {
        snprintf(line, LINELEN, "no last run info");
      }
      panel.displayRunning(line); 
      tDisplayRunning.setInterval(RUNNING_DISPLAY_START_DELAY);
      tDisplayRunning.enableDelayed();
      
      showHumidity(TDISPLAY_SHORT_DELAY * TASK_SECOND);
      return;
      
    case DLOG_P:
      return;
      
    case DERROR:
      break;
  }
  if (!tDisplayRunning.isEnabled()) { 
    panel.displayString(line);
    panelOn();
    if (isLeak()) {
      panel.setDot(0, true);
      panel.setDot(1, true);
      panel.setDot(2, true);
      panel.setDot(3, true);
    }
  }
  
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.print(F(": displayCallback. line="));
  Serial.println(line);
#endif
}


void showHumidity(int aDelay) {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": showHumidity"));
#endif  
  switchDisplayNow(DHUMIDITY, aDelay);
}


void switchDisplayNow(int aStatus, int aDelay) {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.print(F(": switchDisplayNow. delay="));
  Serial.println(aDelay);
#endif  
  displayNow = (Display_Options) aStatus;
  if (aDelay == 0) {
    tDisplayRunning.disable();
    tDisplay.enable(); 
  }
  else {
    tDisplay.delay(aDelay);
  }
}

void displayRunningCallback() {
  tDisplayTimeout.delay();
  tSettingsTimeout.delay();
  tDisplayRunning.setInterval(RUNNING_DISPLAY_DELAY);
  if (!panel.displayRunningShift()) {
    tDisplay.delay(TDISPLAY_SHORT_DELAY * TASK_SECOND);
    tDisplayRunning.disable();
  }
  panelOn();
}



bool isNight() {
  time_t tnow = myTZ.toLocal( now() );

  int hr = hour(tnow);
  int wkp = parameters.wakeup;

//  Add adjusting hours to the wakeup time for Saturday and Sunday  
  if (weekday(tnow) == dowSunday || weekday(tnow) == dowSaturday) wkp += parameters.wkendadj;
  
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.print(F(": isNight. hr="));
  Serial.println(hr);
#endif

  return (hr >= parameters.gotosleep || hr < wkp);
}



void goodnightCallbackInit() {

  night_time = !isNight();
  tGoodnight.setCallback(&goodnightCallback);
  tGoodnight.delay();
  goodnightCallback();
}

void goodnightCallback() {

  checkUpdateTime();
  
  time_t tnow = myTZ.toLocal(now());
  int  hr = hour(tnow);

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": goodnightCallback."));
#endif
  switch (hr) {
    case 7:
      panel_brightness = 3; 
      break;
    case 8:
      panel_brightness = 5;
      break;
    case 9:
      panel_brightness = 6;
      break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
      panel_brightness = 7;
      break;
    case 18:
      panel_brightness = 6;
      break;
    case 19:
      panel_brightness = 4;
      break;
    case 20:
      panel_brightness = 3;
      break;
    default:
      panel_brightness = 1;
  }
  panel.setBrightness(panel_brightness);
  
  if (!night_time && isNight() ) {
    night_time = true;
    motorOff();
    switchDisplayNow(DGOODNIGHT, 0);
    tMeasure.disable();
    measurePower(false);
    tWater.disable();
  }
  if (night_time) {
    if (!isNight() ) {
      night_time = false;
      measurePowerupCallback();
      tDisplay.enable();
      showHumidity(0);
    }
    else {
      if (displayNow != DEMPTY) switchDisplayNow(DGOODNIGHT, 0); //switchDisplayNow(DEMPTY, 0);
    }
  }
}

int pressCnt = 0;
void interruptCallback() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": interruptCallback."));
#endif
  enableInterrupt(BTN_SEL_PIN, &initButtons, RISING); 
  enableInterrupt(BTN_PLUS_PIN, &initButtons, RISING); 
  enableInterrupt(BTN_MINUS_PIN, &initButtons, RISING); 
  pressCnt = 0;
}

void initButtons() {
  disableInterrupt(BTN_SEL_PIN); 
  disableInterrupt(BTN_PLUS_PIN); 
  disableInterrupt(BTN_MINUS_PIN);
    
  tIsr.setInterval(BTN_INIT);
  tIsr.enableDelayed(BNT_DEBOUNCE);  
}

void buttonRelease() {
  disableInterrupt(BTN_SEL_PIN); 
  disableInterrupt(BTN_PLUS_PIN); 
  disableInterrupt(BTN_MINUS_PIN);
  
  tIsr.disable();
  tInterrupt.restartDelayed(BNT_DEBOUNCE);
}

void buttonISR() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": buttonISR."));
#endif

  if (pressCnt == 0) {
    enableInterrupt(BTN_SEL_PIN, &buttonRelease, FALLING); 
    enableInterrupt(BTN_PLUS_PIN, &buttonRelease, FALLING); 
    enableInterrupt(BTN_MINUS_PIN, &buttonRelease, FALLING); 
    tIsr.setInterval(BTN_REPEAT);
    pressCnt++;
  }

  if (pressCnt == BNT_RPT_CNT) {
    tIsr.setInterval(BTN_RAPID);
  }

  if (!panel_status) {
#ifdef _DEBUG_
//  Serial.println(F("Panel On"));
#endif

    switchDisplayNow((night_time ? DEMPTY : DHUMIDITY), 0);
  }
  else {
    buttonsCallback();
  }
}


void settingsToutCallback() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": settingsToutCallback."));
#endif
  loadParameters();
  displayOnlyDoTout();
  tMeasure.enableDelayed();
  switchDisplayNow(DCANCELLED, 0);
}

void settingsDoTout() {
  loadParameters();
  goodnightCallbackInit();  // in case the clock was adjusted

  displayOnlyDoTout();
}

void displayOnlyDoTout() {
  tSettingsTimeout.disable();
  tDisplay.enable();
  tDisplayTimeout.enableDelayed();
  tGoodnight.enableDelayed();
#ifdef _TEST_
  tTestHumidity.enable();
#endif
}


void buttonsCallback() {
  bool sel, plus, minus;
  int value, increment = 0;
  time_t tnow;

  

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": buttonsCallback."));
#endif

    sel = plus = minus = false;
    if ( !(sel = pSelect) ) {
      plus = pPlus;
      minus = pMinus;

      if (plus && minus) plus = minus = false;
    }

#ifdef _DEBUG_
  Serial.print(F("sel/plus/minus="));Serial.print(sel);Serial.print(plus);Serial.println(minus);
  Serial.print(F("repeat Count="));Serial.println(pressCnt);
#endif

  if (plus) increment = 1;
  if (minus) increment = (-1);

  if (sel) {
    switch (displayNow) {
      case DHUMIDITY:
      case DEMPTY:
        switchDisplayNow(DSET, 0);
        
        measurePower(false);
        
        ts.disableAll();
        tDisplay.enable();
        tIsr.enableDelayed();
        
        tSettingsTimeout.set(TSETTOUT_INTERVAL * TASK_SECOND, 1, &settingsToutCallback);
        tSettingsTimeout.enableDelayed();
        
        adjParam = &parameters.high;
        paramIndex = PARAMIDXSTART;
        break;
        
      case DSET:
        switchDisplayNow(DSET_UL, 0);
        break;

      case DCLK:
        tnow = myTZ.toLocal( now() );
        calcTime(tnow);
        adjParam = &mytime.cn;
        paramIndex = CLOCKIDXSTART;
        switchDisplayNow(DCLK_CN, 0);
        break;

      case DLOG:
        paramIndex = 0;
        readLogEntry(paramIndex);
        switchDisplayNow(DLOG_V, 0);
        break;

      case DFRN:
          tWater.setInterval( parameters.saturate * TASK_MINUTE );
          tWater.setIterations( parameters.retries + 1 );         
          tWater.enable();
          tMeasure.set(TMEASURE_WATERTIME * TASK_SECOND, -1, &measureCallback);
          tMeasure.enableDelayed();  
          error = false;
          settingsDoTout();
          switchDisplayNow(DCLK_SET, 0);
          return;         
          break;
        
        
      case DSET_UL:
      case DSET_LL:
      case DSET_PT:
      case DSET_RT:
      case DSET_ST:
      case DSET_SL:
      case DSET_UP:
        switchDisplayNow (displayNow + 1, 0);
        adjParam++;
        paramIndex++;
        break;

      case DLOG_V:
      case DLOG_P:
        displayOnlyDoTout();
        tMeasure.enableDelayed();
        switchDisplayNow(DHUMIDITY, 0);
        return;
        
      case DSET_AD:
        saveParameters();
        settingsDoTout();
        switchDisplayNow(DSTORED, 0);
        return;

      case DCLK_CN:
      case DCLK_YR:
      case DCLK_MT:
      case DCLK_DY:
      case DCLK_HR:
        switchDisplayNow (displayNow + 1, 0);
        adjParam++;
        paramIndex++;
        break;

      case DCLK_MN:
        setTimeNow();
        settingsDoTout();
        switchDisplayNow(DCLK_SET, 0);
        return;
    }
  }

  if (plus || minus) {
    switch (displayNow) {
      case DHUMIDITY:
      case DEMPTY:
        if (plus) {
          switchDisplayNow(DTIME, 0);
        }
        else {
           if (logCount) switchDisplayNow(DRUN, 0);
        }
        return;
        break;
        
      case DSET:
        switchDisplayNow(DCLK, 0);
        break;

      case DCLK:
        switchDisplayNow(DFRN, 0);
        break;

      case DFRN:
        if (logCount) switchDisplayNow(DLOG, 0);
        else switchDisplayNow(DSET, 0);
        break;

      case DLOG:
        switchDisplayNow(DSET, 0);
        break;
        
      case DLOG_V:
      case DLOG_P:
        paramIndex += increment;
        if (paramIndex < 0) paramIndex = logCount;
        if (paramIndex >= logCount) paramIndex = 0;
        readLogEntry(paramIndex);
        switchDisplayNow(DLOG_V, 0);
        break;
        
      case DSET_UL:
      case DSET_LL:
      case DSET_PT:
      case DSET_ST:
      case DSET_RT:
      case DSET_SL:
      case DSET_UP:
      case DSET_AD:
      case DCLK_CN:
      case DCLK_YR:
      case DCLK_MT:
      case DCLK_DY:
      case DCLK_HR:
      case DCLK_MN:
        if (displayNow > DCLK) {
          switch (mytime.mt) {
            case 2:
              paramMax[DAYSIDX] = isLeapYear(mytime.cn * 100 + mytime.yr) ? 29 : 28;
              break;

            case 4:
            case 6:
            case 9:
            case 11:
              paramMax[DAYSIDX] = 30;
              break;

            default:
              paramMax[DAYSIDX] = 31;
          }
        }
        value = (int) * adjParam + increment;
        if (value > (int) paramMax[paramIndex]) value = (int) paramMin[paramIndex];
        if (value < (int) paramMin[paramIndex]) value = (int) paramMax[paramIndex];
        *adjParam = (byte) value;
        // Special case checks:
        if (parameters.high <= parameters.low) parameters.high = parameters.low + 1;
        if (parameters.wakeup >= parameters.gotosleep) parameters.gotosleep = parameters.wakeup + 1;
        switchDisplayNow(displayNow, 0);
    }
  }

  if (sel || plus || minus) {
#ifdef _DEBUG_
//  Serial.println(F("some button pressed"));
#endif
    tSettingsTimeout.delay();
    pressCnt++;
  }
  else {
#ifdef _DEBUG_
//  Serial.println(F("no buttons pressed"));
#endif
    tIsr.disable();
    tInterrupt.restart();
  }
}

bool isLeapYear (int aYr) {
  return ((aYr % 400 == 0) || (aYr % 100 != 0 && aYr % 4 == 0));
}

//  byte  cn; // century
//  byte  yr; // year
//  byte  mt; // month
//  byte  dy; // day
//  byte  hr; // hour
//  byte  mn; // minute

void calcTime(time_t aDate) {
        mytime.cn = year(aDate) / 100;
        mytime.yr = year(aDate) % 100;
        mytime.mt = month(aDate);
        mytime.dy = day(aDate);
        mytime.hr = hour(aDate);
        mytime.mn = minute(aDate);
}

void setTimeNow() {
#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": setTimeNow."));
#endif

//  rtc.adjust(DateTime(mytime.cn*100 + mytime.yr, mytime.mt, mytime.dy, mytime.hr, mytime.mn, 0));  

//typedef struct  { 
//  uint8_t Second; 
//  uint8_t Minute; 
//  uint8_t Hour; 
//  uint8_t Wday;   // day of week, sunday is day 1
//  uint8_t Day;
//  uint8_t Month; 
//  uint8_t Year;   // offset from 1970; 
//}   tmElements_t

    tmElements_t tm; // = { 0, time.mn, time.hr, 0, time.dy, time.mt, time.cn*100 + time.yr - 1970 };
    
    tm.Second = 0;
    tm.Minute = mytime.mn;
    tm.Hour = mytime.hr;
    tm.Wday = 0;
    tm.Day = mytime.dy;
    tm.Month = mytime.mt;
    tm.Year = mytime.cn*100 + mytime.yr - 1970;
    time_t utc = myTZ.toUTC( makeTime(tm) );
#ifndef _USERTCMILLIS_
    rtc.adjust( DateTime(utc) );  
#endif
    setTime( utc );
}

void errorCallback() {
#ifdef _TEST_
  long cnt = 60L;  // 1 minute in seconds
#else
  long cnt = 86400L; // 24 hours in  seconds
#endif

#ifdef _DEBUG_
  Serial.print(millis());
  Serial.println(F(": errorCallback."));
#endif
  state = HIGH;
  panel.displayString("Errr");
  while (cnt--) {
    // We were not able to achieve desired level in several tries
    // Something is wrong - maybe out of water, regardless, stop and
    // blink for 24 hours, then try again. 
//    state = !state;
    panel.displayState( (state = !state) );
    delay (1000);
  }
//  panel.displayState(HIGH);
//  tError.disable(); 
//  settingsDoTout();  
//  showHumidity(0);

  wdt_enable(WDTO_15MS); // reset the device
  for(;;);
}

time_t getTime() {
    return rtc.now().unixtime();
}

void setPins() {

  pLED = LOW; 
  
#ifndef _USERTCMILLIS_
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); //power for RTC
#endif

  pinMode(LEAKPINPWR, OUTPUT);
  digitalWrite(LEAKPINPWR, LOW);
  pinMode(LEAKPINMSR, INPUT);
}

//#define SET_TIME   1446343140L  // 11/1/2015 1:59:00 AM (1 minutes to DST time change)
#define SET_TIME   1420113600L  // 1/1/2015 12:00:00 PM 
//#define SET_TIME   1543709361L 
void setup () {
  char line[54];

  wdt_disable();
  setPins();
  motorOff();

  Wire.begin(); //Join the bus as master

  panel.init();
  panel.displayString((char*)CToken);
  panelOn();

#ifdef _DEBUG_
  Serial.begin(115200);
  Serial.println(F("Plant Watering System"));
#endif

  measurePowerupCallback();

#ifdef _DEBUG_
//  Serial.println(F("Done: measurePowerupCallback"));
#endif
  
  error = false;
  for (int i = 0; i < 6; i++) {
    currentHumidity = measureHumidity();
    delay(1000);
  }

//#ifdef _DEBUG_
//  Serial.println(F("Done: currentHumidity cycle"));
//#endif

  motorReverse();
  loadParameters();

//#ifdef _DEBUG_
//  Serial.println(F("Done: setup::loadParameters"));
//#endif

  adjParam = &parameters.high;
  paramIndex = 0;

  for (int i = 0; i < 8; i++, adjParam++) {
    snprintf(line, 5, "%c%3d", paramChar[i], *adjParam);
    line[0] |= 0x80;
    panel.displayString(line);
    delay(2000);
  }

  motorOff();
  
//#ifndef _USERTCMILLIS_
  rtc.begin();
  if (!rtc.isrunning()) {
    rtc.adjust( DateTime(myTZ.toUTC(SET_TIME)) );   
    delay(1000); // to settle   
  }
  if (!rtc.isrunning()) {
    panel.displayString("rtcE");
    while (1) {}
  }
  setSyncProvider(&getTime);   // the function to get the time from the RTC
  checkUpdateTime();
  setSyncInterval(600);      // Sync time every ten minutes with RTC
  if( timeStatus() != timeSet ) {
    panel.displayString("rtcE");
    while (1) {}
  }
#ifdef _USERTCMILLIS_
  rtc.adjust( DateTime(myTZ.toUTC(SET_TIME)) );
  setTime( myTZ.toUTC(SET_TIME) ); 
#endif

  time_t tnow = myTZ.toLocal( now() );


  snprintf( line, 54, "date %04d-%02d-%02d %02d%02d", year(tnow), month(tnow), day(tnow), hour(tnow), minute(tnow) );
  line[17] |= 0b10000000;  //dot
  panel.displayRunning(line); delay(RUNNING_DISPLAY_START_DELAY);
  while (panel.displayRunningShift()) delay(RUNNING_DISPLAY_DELAY);
  delay(2000);

#ifdef _TEST_
  panel.displayString("TEST");
  for (int i=0; i<12; i++) {
    panel.displayState(i&1);
    delay(500);
  }
  delay(1000);
#endif

#ifdef _DEBUG_
  panel.displayString("Debg");
  for (int i=0; i<12; i++) {
    panel.displayState(i&1);
    delay(500);
  }
  delay(1000);
#endif

  tInterrupt.enable();
  tDisplayTimeout.enableDelayed();
  showHumidity(0);
}


void loop ()
{
  ts.execute();
}


