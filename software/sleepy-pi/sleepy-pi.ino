// INDENTING (emacs/vi): -*- mode:c++; tab-width:2; c-basic-offset:2; intent-tabs-mode:nil; -*- ex: set tabstop=2 expandtab:

//
// This Arduino sketch programs the Sleepy Pi 2 such as to provide:
//
// * Button control:
//   - short press (< 2 seconds): power the Raspberry Pi up
//   - short hold  (< 8 seconds): shut  the Raspberry Pi down (gracefully)
//   - long  hold  (> 8 seconds): power the Raspberry Pi down (forcefully)
//                (warning: data loss and filesystem corruption may occur)
//
// * Wake on RTC (programmable via I2C; see below):
//   - wake on timer: on given elapsed day, hour and minute
//   - wake on alarm: on given calendar day, hour and minute
//
// * Voltage control:
//   - shut the Raspberry Pi down on low voltage
//
// * (optional) Temperature control
//   - shut the Raspberry Pi down on high temperature
//
// * (optional) Expansion power control
//   - power Expansion along the Raspberry Pi
//   OR
//   - power Expansion along the temperature
//
// * I2C slave, on address 0x04:
//   - read:
//     > Sleepy Pi voltage [mV]    : command 0x01, request 2 bytes
//     > Raspberry Pi current [mA] : command 0x02, request 2 bytes
//     > Temperature [cK]          : command 0x02, request 2 bytes
//     > Refresh                   : command 0x80
//   - write:
//     > Shut-down                 : command 0x81
//     > Shut-down + Wake-on-Timer : command 0x82, send 1(2,3) bytes: minute, (hour), (day)
//     > Shut-down + Wake-on-Alarm : command 0x83, send 2(3) bytes: hour, minute, (day)
//   to read:
//   - send the Refresh command;
//     > i2cset -y 1 0x04 0x80
//   - wait 250ms (for the Sleepy Pi to wake-up and perform ADC readings)
//   - request the desired data (within 500ms of the command); e.g read Sleepy Pi voltage
//     > i2cget -y 1 0x04 0x01 w
//   to write:
//   - send the desired command/data; e.g. Wake-on-Alarm at 08h05 (UTC)
//     > i2cset -y 1 0x04 0x82 0x08 0x05 i
//

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// LIBRARIES
//

// Sleepy Pi 2
// REF: https://github.com/SpellFoundry/SleepyPi2/
#include "SleepyPi2.h"

// Sleepy Pi 2 Real-Time Clock (RTC) - PCF8523
// REF: https://github.com/SpellFoundry/PCF8523
#include <PCF8523.h>

// Time
// REF: https://github.com/PaulStoffregen/Time
#include <TimeLib.h>

// Low-Power
// REF: https://github.com/rocketscream/Low-Power
#include <LowPower.h>

// I2C
// REF: https://www.arduino.cc/en/Reference/Wire
#include <Wire.h>



////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CONSTANTS
//

// Debug (enable Serial output)
#define DEBUG false

// Sleepy Pi LED pin
#define LED_PIN 13

// Analogic-Digital Converter (ADC) reference voltage [V]
#define ADC_REFERENCE_VOLTAGE 3.3f


//
// Options
//

// Automatically power the Raspberry Pi on when power supply is switched on
#define POWER_AUTO false

// Temperature ADC pin; disable: 0 / enable: 14 (A0) or 15 (A1)
#define TEMPERATURE_PIN 0

// Raspberry Pi threshold current [mA] (power down <-> power up)
#define POWER_CURRENT_THRESHOLD 90.0f

// Power the Raspberry Pi on/off depending on the environment (voltage/temperature)
#define POWER_CONTROL false
// ... low Sleepy Pi supply voltage [V]
#define POWER_CONTROL_VOLTAGE_LOW  11.5f  // ~ 10% capacity
#define POWER_CONTROL_VOLTAGE_HIGH 12.3f  // ~ 50% capacity
// ... high temperature [K]
#define POWER_CONTROL_TEMPERATURE_HIGH 323.16f  // 50°C
#define POWER_CONTROL_TEMPERATURE_LOW  313.16f  // 40°C

// Expansion power (e.g. fan)
#define EXPANSION_ENABLE false

// Expansion temperatures (on <-> off) [K]
#define EXPANSION_TEMPERATURE     false
#define EXPANSION_TEMPERATURE_ON  303.16f  // 30°C
#define EXPANSION_TEMPERATURE_OFF 298.16f  // 25°C


//
// I2C
//

// I2C (slave) address
#define I2C_ADDRESS 0x04

// I2C commands
#define I2C_OPCODE_NONE          0x00
#define I2C_OPCODE_VOLTAGE_R     0x01
#define I2C_OPCODE_CURRENT_R     0x02
#define I2C_OPCODE_TEMPERATURE_R 0x03
#define I2C_OPCODE_REFRESH       0x80
#define I2C_OPCODE_SHUTDOWN_W    0x81
#define I2C_OPCODE_WAKEONTIMER_W 0x82
#define I2C_OPCODE_WAKEONALARM_W 0x83
#define I2C_OPCODE_DETECT        0xFF

// I2C values
#define I2C_VALUE_UNSET 0xFF


//
// Sleepy Pi Button
//

// Sleepy Pi Button interrupt (<-> pin 3, aka POWER_BUTTON_PIN)
#define BUTTON_INT 1

// Button state tracking
#define BUTTON_STATE_RELEASED 0
#define BUTTON_STATE_PRESSED  1

// Button elapsed state tracking [ms]
#define BUTTON_ELAPSED_SHUTDOWN 2000
#define BUTTON_ELAPSED_POWEROFF 8000


//
// Sleepy Pi RTC
//

// Sleepy Pi RTC interrupt (<-> pin 2, aka ALARM_PIN)
#define RTC_INT 0

// RTC state tracking
#define RTC_STATE_UNSET 0
#define RTC_STATE_TIMER 1
#define RTC_STATE_ALARM 2
#define RTC_STATE_SET   3



////////////////////////////////////////////////////////////////////////////////////////////////////
//
// GLOBALS
//

// Raspberry Pi power
bool bPiPowered;
bool bPiShutdown;
bool bPiPoweroff;
bool bPiPoweron;

// Expansion power
bool bExpansionPowered;
bool bExpansionPoweroff;
bool bExpansionPoweron;

// Environment
float fVoltage;      // [V]
float fCurrent;      // [mA]
float fTemperature;  // [K]

// Power control
bool bPowerControlVoltage;
bool bPowerControlTemperature;

// Button state tracking
volatile bool bButtonInterrupted;
int iButtonState;
unsigned long ulButtonPressedTime;

// RTC state tracking
volatile bool bRtcInterrupted;
int iRtcState;

// I2C state tracking
volatile bool bI2cInterrupted;
volatile uint8_t yI2cOpCode;
volatile uint8_t yI2cValDay;
volatile uint8_t yI2cValHour;
volatile uint8_t yI2cValMinute;



////////////////////////////////////////////////////////////////////////////////////////////////////
//
// FUNCTIONS
//

//
// Interrupt Service Routines (ISRs)
// NOTE:
// - processing in ISR functions MUST be kept to a minimum (don't hold interrupts up)
// - global variables that are modified in ISR functions MUST be declared volatile
//

// Sleepy Pi Button
void isrButton() {
  bButtonInterrupted = true;
}

// Sleepy Pi RTC
void isrRtc() {
  bRtcInterrupted = true;
}

// I2C Receive
void isrI2cReceive(int iBytes) {
  bI2cInterrupted = true;
  yI2cOpCode = iBytes-- > 0 ? Wire.read() : I2C_OPCODE_NONE;
  switch(yI2cOpCode) {

  case I2C_OPCODE_WAKEONTIMER_W:
    yI2cValMinute = iBytes-- > 0 ? Wire.read() : I2C_VALUE_UNSET;
    yI2cValHour = iBytes-- > 0 ? Wire.read() : I2C_VALUE_UNSET;
    yI2cValDay = iBytes-- > 0 ? Wire.read() : I2C_VALUE_UNSET;
    break;

  case I2C_OPCODE_WAKEONALARM_W:
    yI2cValHour = iBytes-- > 0 ? Wire.read() : I2C_VALUE_UNSET;
    yI2cValMinute = iBytes-- > 0 ? Wire.read() : I2C_VALUE_UNSET;
    yI2cValDay = iBytes-- > 0 ? Wire.read() : I2C_VALUE_UNSET;
    break;

  default:
    break;

  }
}

// I2C Request
void isrI2cRequest() {
  bI2cInterrupted = true;

  // OpCode -> Value
  uint32_t uiValue;
  int iBytes;
  switch(yI2cOpCode) {

  case I2C_OPCODE_VOLTAGE_R:
    uiValue = (uint32_t)(1000.0f*fVoltage);  // [mV]
    iBytes = 2;
    break;

  case I2C_OPCODE_CURRENT_R:
    uiValue = (uint32_t)fCurrent;  // [mA]
    iBytes = 2;
    break;

  case I2C_OPCODE_TEMPERATURE_R:
    uiValue = (uint32_t)(100.0f*fTemperature);  // [cK]
    iBytes = 2;
    break;

  default:
    uiValue = 0U;
    iBytes = 1;
    break;

  }
  yI2cOpCode = I2C_OPCODE_NONE;

  // Send
  char pValue[4];
  for(int i=iBytes; i>=0; i--) {
    pValue[i] = (uint8_t)(uiValue >> 8*i);
  }
  Wire.write(pValue, iBytes);
}


//
// Input/Output
//

void button() {
  detachInterrupt(BUTTON_INT);
  bButtonInterrupted = false;

  switch(iButtonState) {

  case BUTTON_STATE_RELEASED:
    iButtonState = BUTTON_STATE_PRESSED;
    digitalWrite(LED_PIN, HIGH);
    ulButtonPressedTime = millis();
    attachInterrupt(BUTTON_INT, isrButton, RISING);
    break;

  case BUTTON_STATE_PRESSED: {
    iButtonState = BUTTON_STATE_RELEASED;
    digitalWrite(LED_PIN, LOW);
    unsigned long ulButtonPressedTimeElapsed = millis() - ulButtonPressedTime;
    if(ulButtonPressedTimeElapsed > BUTTON_ELAPSED_POWEROFF) {
      bPiPoweroff = true;
    }
    else if (ulButtonPressedTimeElapsed > BUTTON_ELAPSED_SHUTDOWN) {
      bPiShutdown = true;
    }
    else {
      bPiPoweron = true;
    }
    attachInterrupt(BUTTON_INT, isrButton, FALLING);
    break;
  }

  default:
    break;

  }

#if DEBUG
  Serial.print("Button: ");
  Serial.println(iButtonState == BUTTON_STATE_PRESSED ? "pressed" : "released");
#endif
}

void i2c() {
#if DEBUG
  Serial.print("I2C Command: ");
  Serial.println(yI2cOpCode);
  if(yI2cOpCode > I2C_OPCODE_REFRESH) {
    Serial.print("I2C Values (day, hour, minute): ");
    Serial.print(yI2cValDay);
    Serial.print(", ");
    Serial.print(yI2cValHour);
    Serial.print(", ");
    Serial.println(yI2cValMinute);
  }
#endif

  // I2C command
  switch(yI2cOpCode) {

  case I2C_OPCODE_SHUTDOWN_W:
    bPiShutdown = true;
    break;

  case I2C_OPCODE_WAKEONTIMER_W:
    iRtcState = RTC_STATE_TIMER;
    if(yI2cValDay == I2C_VALUE_UNSET) {
      yI2cValDay = 0;
    }
    if(yI2cValHour == I2C_VALUE_UNSET) {
      yI2cValHour = 0;
    }
    if(yI2cValMinute == I2C_VALUE_UNSET) {
      yI2cValHour = 0;
    }
    if(yI2cValDay == 0 and yI2cValHour == 0 and yI2cValMinute == 0) {
      iRtcState = RTC_STATE_UNSET;
    }
    if(iRtcState != RTC_STATE_UNSET) {
      bPiShutdown = true;
    }
    break;

  case I2C_OPCODE_WAKEONALARM_W:
    iRtcState = RTC_STATE_ALARM;
    if(yI2cValDay != I2C_VALUE_UNSET and (yI2cValDay < 1 or yI2cValDay > 31)) {
      iRtcState = RTC_STATE_UNSET;
    }
    if(yI2cValHour == I2C_VALUE_UNSET or yI2cValHour > 23) {
      iRtcState = RTC_STATE_UNSET;
    }
    if(yI2cValMinute == I2C_VALUE_UNSET or yI2cValMinute > 59) {
      iRtcState = RTC_STATE_UNSET;
    }
    if(iRtcState != RTC_STATE_UNSET) {
      bPiShutdown = true;
    }
    break;

  case I2C_OPCODE_REFRESH:
    // A request ought to come soon
    delay(1000);
    break;

  default:
    break;

  }

  yI2cOpCode = I2C_OPCODE_NONE;
}


//
// RTC
//

void rtcSet() {
#if DEBUG
  Serial.print("RTC Set (state): ");
  Serial.println(iRtcState);
  Serial.print("RTC Values (day, hour, minute): ");
  Serial.print(yI2cValDay);
  Serial.print(", ");
  Serial.print(yI2cValHour);
  Serial.print(", ");
  Serial.println(yI2cValMinute);
#endif

  // Enable the RTC trigger
  switch(iRtcState) {

  case RTC_STATE_TIMER: {
    DateTime dtNow = SleepyPi.readTime();
    DateTime dtAlarm(dtNow.unixtime() + 86400L*yI2cValDay + 3600L*yI2cValHour + 60L*yI2cValMinute);
    SleepyPi.setAlarm(dtAlarm.day(), dtAlarm.hour(), dtAlarm.minute());
    break;
  }

  case RTC_STATE_ALARM:
    if(yI2cValDay == I2C_VALUE_UNSET) {
      SleepyPi.setAlarm(yI2cValHour, yI2cValMinute);
    }
    else {
      SleepyPi.setAlarm(yI2cValDay, yI2cValHour, yI2cValMinute);
    }
    break;

  default:
    iRtcState = RTC_STATE_UNSET;
    return;
  }
  SleepyPi.enableAlarm(true);
  iRtcState = RTC_STATE_SET;
  yI2cValDay = yI2cValHour = yI2cValMinute = I2C_VALUE_UNSET;

  // Enable interrupt: Sleepy Pi RTC
  SleepyPi.rtcClearInterrupts();
  attachInterrupt(RTC_INT, isrRtc, FALLING);

#if POWER_CONTROL or (EXPANSION_ENABLE and EXPANSION_TEMPERATURE and TEMPERATURE_PIN)
  // QUIRK: The first occurence of the WDT interrupt is mistaken for an RTC/INT0 one (?!?)
  SleepyPi.powerStandby(SLEEP_15MS, ADC_ON, BOD_ON);
  bRtcInterrupted = false;
#endif
}

void rtcUnset() {
#if DEBUG
  Serial.println("RTC Unset");
#endif

  // Disable interrupt: Sleepy Pi RTC
  detachInterrupt(RTC_INT);

  // Acknowledge the RTC trigger
  SleepyPi.ackAlarm();
  SleepyPi.enableAlarm(false);
  iRtcState = RTC_STATE_UNSET;
}


//
// I2C
//

void i2cSlave() {
#if DEBUG
  Serial.println("I2C Begin (slave)");
#endif

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(isrI2cReceive);
  Wire.onRequest(isrI2cRequest);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MAIN
//

//
// Setup
//

void setup() {
#if DEBUG
  // Initialize serial line <-> Arduino IDE "Serial Monitor"
  Serial.begin(115200);
#endif

  // Sleepy Pi LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Environment
  fCurrent = SleepyPi.rpiCurrent();
#if not TEMPERATURE_PIN
  fTemperature = 0.0f;
#endif

  // Power control
  bPowerControlVoltage = false;
  bPowerControlTemperature = false;

  // Raspberry Pi status
  bPiPowered = fCurrent > POWER_CURRENT_THRESHOLD;
#if POWER_AUTO
  if(not bPiPowered) {
    SleepyPi.enablePiPower(true);
    delay(1000);
    bPiPowered = true;
  }
#endif

#if EXPANSION_ENABLE
#if not (EXPANSION_TEMPERATURE and TEMPERATURE_PIN)
  // Power Expansion along the Raspberry Pi
  bExpansionPowered = bPiPowered;
#else
  // Temperature-controlled
  bExpansionPowered = false;
#endif
  SleepyPi.enableExtPower(bExpansionPowered);
#else
  // Power Expansions down
  SleepyPi.enableExtPower(false);
#endif

  // Button state tracking
  bButtonInterrupted = false;
  iButtonState = BUTTON_STATE_RELEASED;
  ulButtonPressedTime = 0;

  // Enable interrupt: Sleepy Pi Button
  attachInterrupt(BUTTON_INT, isrButton, FALLING);

  // Initialize the RTC
  SleepyPi.rtcInit(true);

  // RTC state tracking
  bRtcInterrupted = false;
  iRtcState = RTC_STATE_UNSET;

  // I2C (Raspberry Pi <-> Sleepy Pi)
  yI2cOpCode = I2C_OPCODE_NONE;
  yI2cValDay = yI2cValHour = yI2cValMinute = I2C_VALUE_UNSET;
  i2cSlave();
}


//
// Loop
//

void loop() {
  // Raspberry Pi status
  bPiShutdown = false;
  bPiPoweroff = false;
  bPiPoweron = false;

  // Button state tracking
  if(bButtonInterrupted) {
    button();
  }
  bButtonInterrupted = false;
  if(iButtonState == BUTTON_STATE_PRESSED) {
    delay(250);
    goto endLoop;
  }

  // RTC state tracking
  if(bRtcInterrupted) {
    bPiPoweron = true;
  }
  bRtcInterrupted = false;

  // Environment
  fVoltage = SleepyPi.supplyVoltage();
  fCurrent = SleepyPi.rpiCurrent();
#if TEMPERATURE_PIN
  // (TMP36 sensor: 10mv/°C ; -50°C/223.16°K at 0V)
  // TODO: Fix fluctuating temperature readings when Pi is on (up to 5 degrees less than when standing by)
  fTemperature = analogRead(TEMPERATURE_PIN) * ADC_REFERENCE_VOLTAGE / 1023.0f * 100.0f + 223.16f;  // [K]
#endif
#if DEBUG
  Serial.print("Voltage [V]: ");
  Serial.println(fVoltage);
  Serial.print("Current [mA]: ");
  Serial.println(fCurrent);
  Serial.print("Temperature [K]: ");
  Serial.println(fTemperature);
#endif

  // I2C
  if(yI2cOpCode > I2C_OPCODE_NONE) {
    i2c();
  }
  bI2cInterrupted = false;

  // Raspberry Pi status (cont'd)
  bPiPowered = fCurrent > POWER_CURRENT_THRESHOLD;
#if DEBUG
  Serial.print("Raspberry Pi: ");
  Serial.println(bPiPowered ? "on" : "off");
#endif

  // Expansion status
  bExpansionPoweroff = false;
  bExpansionPoweron = false;

  // Environment-based power control
#if POWER_CONTROL
  if(fVoltage < (bPowerControlVoltage ? POWER_CONTROL_VOLTAGE_HIGH : POWER_CONTROL_VOLTAGE_LOW) and (bPiPowered or bPiPoweron)) {
#if DEBUG
    Serial.println("Environment: low voltage shutdown");
#endif
    bPowerControlVoltage = true;
    bPiShutdown = true;
    // Attempt to power-on in one hour and see what gives
    if(iRtcState == RTC_STATE_UNSET) {
      iRtcState = RTC_STATE_TIMER;
      yI2cValMinute = 0;
      yI2cValHour = 1;
      yI2cValDay = 0;
    }
  }
#if TEMPERATURE_PIN
  if(fTemperature > (bPowerControlTemperature ? POWER_CONTROL_TEMPERATURE_HIGH : POWER_CONTROL_TEMPERATURE_HIGH) and (bPiPowered or bPiPoweron)) {
#if DEBUG
    Serial.println("Environment: high temperature shutdown");
#endif
    bPowerControlTemperature = true;
    bPiShutdown = true;
    // Attempt to power-on in one hour and see what gives
    if(iRtcState == RTC_STATE_UNSET) {
      iRtcState = RTC_STATE_TIMER;
      yI2cValMinute = 0;
      yI2cValHour = 1;
      yI2cValDay = 0;
    }
  }
#endif
#endif

  // Raspberry Pi power
  if(bPiShutdown or bPiPoweroff) {
    if(bPiPowered) {
      if(bPiPoweroff) {
#if DEBUG
        Serial.println("Raspberry Pi: power off");
#endif
        SleepyPi.enablePiPower(false);
      }
      else {
#if DEBUG
        Serial.println("Raspberry Pi: shut down");
#endif
        SleepyPi.piShutdown();
      }
      bPiPowered = false;
#if EXPANSION_ENABLE and not (EXPANSION_TEMPERATURE and TEMPERATURE_PIN)
      // Power Expansion down along the Raspberry Pi
      bExpansionPoweroff = true;
#endif
    }
  }
  else if(bPiPoweron) {
    if(not bPiPowered) {
#if DEBUG
      Serial.println("Raspberry Pi: power on");
#endif
      SleepyPi.enablePiPower(true);
      bPiPowered = true;
#if EXPANSION_ENABLE and not (EXPANSION_TEMPERATURE and TEMPERATURE_PIN)
      // Power Expansion up along the Raspberry Pi
      bExpansionPoweron = true;
#endif
    }
    bPowerControlVoltage = false;
    bPowerControlTemperature = false;
  }

  // Expansion power
#if EXPANSION_ENABLE and EXPANSION_TEMPERATURE and TEMPERATURE_PIN
  // Temperature-controlled
  if(fTemperature > EXPANSION_TEMPERATURE_ON) {
    bExpansionPoweron = true;
  }
  else if(fTemperature < EXPANSION_TEMPERATURE_OFF) {
    bExpansionPoweroff = true;
  }
#endif
#if EXPANSION_ENABLE
  if(bExpansionPoweron) {
    if(not bExpansionPowered) {
#if DEBUG
      Serial.println("Expansion: power on");
#endif
      SleepyPi.enableExtPower(true);
      bExpansionPowered = true;
    }
  }
  else if(bExpansionPoweroff) {
    if(bExpansionPowered) {
#if DEBUG
      Serial.println("Expansion: power off");
#endif
      SleepyPi.enableExtPower(false);
      bExpansionPowered = false;
    }
  }
#endif

  // Set RTC before sleep
  if(iRtcState and iRtcState != RTC_STATE_SET) {
    rtcSet();  // <-> I2C (internally)
  }

  // Enable I2C slave
  i2cSlave();

  // Sleep
#if DEBUG
  Serial.println("SLEEP");
  delay(250);  // allow Serial.print-ed content to reach the line
#endif
  // Power the Sleepy Pi down
  // - Disable the Analog/Digital Converter (ADC)
  // - Disable the Brown-Out Detection (BOD; low-voltage detection)
#if POWER_CONTROL or (EXPANSION_ENABLE and EXPANSION_TEMPERATURE and TEMPERATURE_PIN)
  SleepyPi.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
#else
  SleepyPi.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
#endif
  // (code execution is halted here)

  // Awoken (interrupt-ed)
#if DEBUG
  Serial.println("AWOKEN");
#endif
  if(iRtcState and (bButtonInterrupted or bRtcInterrupted)) {
    rtcUnset();  // <-> I2C (internally)
  }

endLoop:;
#if DEBUG
  Serial.print("Button Interrupted: ");
  Serial.println(bButtonInterrupted ? "yes" : "no");
  Serial.print("RTC Interrupted: ");
  Serial.println(bRtcInterrupted ? "yes" : "no");
  Serial.print("I2C Interrupted: ");
  Serial.println(bI2cInterrupted ? "yes" : "no");
#endif
}
