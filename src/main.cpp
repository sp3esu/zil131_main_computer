#include <Arduino.h>

/* RC engine sound & LED controller for Arduino ESP32. Written by TheDIYGuy999
    Based on the code for ATmega 328: https://github.com/TheDIYGuy999/Rc_Engine_Sound

 *  ***** ESP32 CPU frequency must be set to 240MHz! *****
    ESP32 macOS Big Sur fix see: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/BigSurFix.md

   Sound files converted with: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/Audio2Header.html
   Original converter code by bitluni (send him a high five, if you like the code)

   Parts of automatic transmision code from Wombii's fork: https://github.com/Wombii/Rc_Engine_Sound_ESP32
*/

const float codeVersion = 6.63; // Software revision.

//
// =======================================================================================================
// ! ! I M P O R T A N T ! !   SETTINGS (ADJUST THEM BEFORE CODE UPLOAD), REQUIRED ESP32 BOARD DEFINITION
// =======================================================================================================
//

// All the required user settings are done in the following .h files:
#include "1_adjustmentsVehicle.h"       // <<------- Select the vehicle you want to simulate
#include "2_adjustmentsRemote.h"        // <<------- Remote control system related adjustments
#include "3_adjustmentsESC.h"           // <<------- ESC related adjustments
#include "4_adjustmentsTransmission.h"  // <<------- Transmission related adjustments
#include "5_adjustmentsShaker.h"        // <<------- Shaker related adjustments
#include "6_adjustmentsLights.h"        // <<------- Lights related adjustments
#include "7_adjustmentsServos.h"        // <<------- Servo output related adjustments

// Install ESP32 board according to: https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
// Adjust board settings according to: https://github.com/TheDIYGuy999/Rc_Engine_Sound_ESP32/blob/master/pictures/settings.png

// Make sure to remove -master from your sketch folder name

// DEBUG options can slow down the playback loop! Only uncomment them for debugging, may slow down your system!
// #define CHANNEL_DEBUG // uncomment it for input signal debugging informations
// #define ESC_DEBUG // uncomment it to debug the ESC
// #define AUTO_TRANS_DEBUG // uncomment it to debug the automatic transmission
//#define MANUAL_TRANS_DEBUG // uncomment it to debug the manual transmission
//#define TRACKED_DEBUG // debugging tracked vehicle mode

// TODO = Things to clean up!

//
// =======================================================================================================
// LIRBARIES & HEADER FILES
// =======================================================================================================
//

// Header files
#include "headers/curves.h" // Nonlinear throttle curve arrays

// Libraries (you have to install all of them in the "Arduino sketchbook"/libraries folder)
// !! Do NOT install the libraries in the sketch folder. This may cause weird rebooting issues !!
#include "statusLED.h" // https://github.com/TheDIYGuy999/statusLED <<------- Install the newest version!
// #include <SBUS.h> // https://github.com/TheDIYGuy999/SBUS you need to install my fork of this library!
// #include <rcTrigger.h> // https://github.com/TheDIYGuy999/rcTrigger <<------- v4.7: This one is now required as well
// #include <IBusBM.h> // https://github.com/bmellink/IBusBM required for IBUS interface

#include "driver/rmt.h" // No need to install this, comes with ESP32 board definition (used for PWM signal detection)
#include "driver/mcpwm.h" // for servo PWM output

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES (Do not play around here)
// =======================================================================================================
//
// Pin assignment and wiring instructions ****************************************************************

// ------------------------------------------------------------------------------------
// Use a 330Ohm resistor in series with all I/O pins! allows to drive LED directly and
// provides short circuit protection. Also works on the serial Rx pin "VP" (36)
// ------------------------------------------------------------------------------------

// Serial command pins for SBUS & IBUS -----
#define COMMAND_RX 36 // pin 36, labelled with "VP", connect it to "Micro RC Receiver" pin "TXO"
#define COMMAND_TX 39 // pin 39, labelled with "VN", only used as a dummy, not connected

// PPM signal pin (multiple channel input with only one wire) -----
#define PPM_PIN 36

// PWM RC signal pins (active, if no other communications profile is enabled) -----
// Channel numbers may be different on your recveiver!
//CH1: (steering)
//CH2: (gearbox) (left throttle in TRACKED_MODE)
//CH3: (throttle) (right throttle in TRACKED_MODE)
//CH4: (horn and bluelight / siren)
//CH5: (high / low beam, transmission neutral, jake brake etc.)
//CH6: (indicators, hazards)
#define PWM_CHANNELS_NUM 6 // Number of PWM signal input pins
const uint8_t PWM_CHANNELS[PWM_CHANNELS_NUM] = { 1, 2, 3, 4, 5, 6}; // Channel numbers
const uint8_t PWM_PINS[PWM_CHANNELS_NUM] = { 13, 12, 14, 27, 35, 34 }; // Input pin numbers

#define ESC_OUT_PIN 33 // connect crawler type ESC here (working fine, but use it at your own risk! Not supported in TRACKED_MODE) -----

#define STEERING_PIN 13 // CH1 output for steering servo (bus communication only)
#define SHIFTING_PIN 12 // CH2 output for shifting servo (bus communication only)
#define COUPLER_PIN 27 // CH4 output for coupler (5th. wheel) servo (bus communication only)

#ifdef PROTOTYPE_36 // switching headlight pin depending on the board variant (do not uncomment it, or it will cause boot issues!)
#define HEADLIGHT_PIN 0 // White headllights connected to pin D0, which only exists on the 36 pin ESP32 board (causes boot issues, if used!)
#else
#define HEADLIGHT_PIN 3 // 3 = RX0 pin, (1 = TX0 is not usable) white headllights
#endif

#define TAILLIGHT_PIN 15 // Red tail- & brake-lights (combined)
#define INDICATOR_LEFT_PIN 2 // Orange left indicator (turn signal) light
#define INDICATOR_RIGHT_PIN 4 // Orange right indicator (turn signal) light
#define FOGLIGHT_PIN 16 // (16 = RX2) Fog lights
#define REVERSING_LIGHT_PIN 17 // (TX2) White reversing light
#define ROOFLIGHT_PIN 5 // Roof lights (high beam, if "define SEPARATE_FULL_BEAM")
#define SIDELIGHT_PIN 18 // Side lights (connect roof ligthts here, if "define SEPARATE_FULL_BEAM")
#define BEACON_LIGHT2_PIN 19 // Blue beacons light
#define BEACON_LIGHT1_PIN 21 // Blue beacons light
#define CABLIGHT_PIN 22 // Cabin lights

#if defined THIRD_BRAKLELIGHT
#define BRAKELIGHT_PIN 32 // Upper brake lights
#else
#define COUPLER_SWITCH_PIN 32 // switch for trailer coupler sound
#endif

#define SHAKER_MOTOR_PIN 23 // Shaker motor (shaking truck while idling and engine start / stop)

#define DAC1 25 // connect pin25 (do not change the pin) to a 10kOhm resistor
#define DAC2 26 // connect pin26 (do not change the pin) to a 10kOhm resistor
// both outputs of the resistors above are connected together and then to the outer leg of a
// 10kOhm potentiometer. The other outer leg connects to GND. The middle leg connects to both inputs
// of a PAM8403 amplifier and allows to adjust the volume. This way, two speakers can be used.

// Objects *************************************************************************************
// Status LED objects (also used for PWM shaker motor and ESC control) -----
statusLED escOut(false);

// Status LED objects (also used for PWM shaker motor and ESC control) -----
statusLED headLight(false); // "false" = output not inversed
statusLED tailLight(false);
// statusLED indicatorL(false);
// statusLED indicatorR(false);
// statusLED fogLight(false);
statusLED reversingLight(false);
statusLED roofLight(false);
// statusLED sideLight(false);
// statusLED beaconLight1(false);
// statusLED beaconLight2(false);
// statusLED shakerMotor(false);
// statusLED cabLight(false);
// statusLED brakeLight(false);


// Global variables **********************************************************************
SemaphoreHandle_t xPwmSemaphore;


// PWM processing variables
#define RMT_TICK_PER_US 1
// determines how many clock cycles one "tick" is
// [1..255], source is generally 80MHz APB clk
#define RMT_RX_CLK_DIV (80000000/RMT_TICK_PER_US/1000000)
// time before receiver goes idle (longer pulses will be ignored)
#define RMT_RX_MAX_US 3500
uint32_t maxPwmRpmPercentage = 400; // Limit required to prevent controller fron crashing @ high engine RPM

// PPM signal processing variables
#define NUM_OF_PPM_CHL 8 // The number of channels inside our PPM signal (8 is the maximum!)
#define NUM_OF_PPM_AVG 1 // Number of averaging passes (usually one, more will be slow)
volatile int ppmInp[NUM_OF_PPM_CHL + 1] = {0}; // Input values
volatile int ppmBuf[NUM_OF_PPM_CHL + 1] = {0}; // Buffered values
volatile byte counter = NUM_OF_PPM_CHL;
volatile byte average  = NUM_OF_PPM_AVG;
volatile boolean ready = false;
volatile unsigned long timelast;
unsigned long timelastloop;
uint32_t maxPpmRpmPercentage = 390; // Limit required to prevent controller fron crashing @ high engine RPM

// Interrupt latches
volatile boolean couplerSwitchInteruptLatch; // this is enabled, if the coupler switch pin change interrupt is detected

// Control input signals
#define PULSE_ARRAY_SIZE 14                              // 13 channels (+ the unused CH0)
uint16_t pulseWidthRaw[PULSE_ARRAY_SIZE];                // Current RC signal RAW pulse width [X] = channel number
uint16_t pulseWidth[PULSE_ARRAY_SIZE];                   // Current RC signal pulse width [X] = channel number
int16_t pulseOffset[PULSE_ARRAY_SIZE];                   // Offset for auto zero adjustment

uint16_t pulseMaxNeutral[PULSE_ARRAY_SIZE];              // PWM input signal configuration storage variables
uint16_t pulseMinNeutral[PULSE_ARRAY_SIZE];
uint16_t pulseMax[PULSE_ARRAY_SIZE];
uint16_t pulseMin[PULSE_ARRAY_SIZE];
uint16_t pulseMaxLimit[PULSE_ARRAY_SIZE];
uint16_t pulseMinLimit[PULSE_ARRAY_SIZE];

uint16_t pulseZero[PULSE_ARRAY_SIZE];                    // Usually 1500 (The mid point of 1000 - 2000 Microseconds)
uint16_t pulseLimit = 1100;                              // pulseZero +/- this value (1100)
uint16_t pulseMinValid = 950;                            // The minimum valid pulsewidth
uint16_t pulseMaxValid = 2050;                           // The maximum valid pulsewidth
bool autoZeroDone;                                       // Auto zero offset calibration done

volatile boolean failSafe = false;                       // Triggered in emergency situations like: throttle signal lost etc.

boolean mode1;                                           // Signal state variables
boolean mode2;
boolean momentary1;
boolean hazard;
boolean left;
boolean right;
boolean unlock5thWheel;

// Sound
volatile boolean engineOn = false;                       // Signal for engine on / off
volatile boolean engineStart = false;                    // Active, if engine is starting up
volatile boolean engineRunning = false;                  // Active, if engine is running
volatile boolean engineStop = false;                     // Active, if engine is shutting down
volatile boolean jakeBrakeRequest = false;               // Active, if engine jake braking is requested
volatile boolean engineJakeBraking = false;              // Active, if engine is jake braking
volatile boolean wastegateTrigger = false;               // Trigger wastegate (blowoff) after rapid throttle drop
volatile boolean dieselKnockTrigger = false;             // Trigger Diesel ignition "knock"
volatile boolean dieselKnockTriggerFirst = false;        // The first  Diesel ignition "knock" per sequence
volatile boolean airBrakeTrigger = false;                // Trigger for air brake noise
volatile boolean parkingBrakeTrigger = false;            // Trigger for air parking brake noise
volatile boolean shiftingTrigger = false;                // Trigger for shifting noise
volatile boolean hornTrigger = false;                    // Trigger for horn on / off
volatile boolean sirenTrigger = false;                   // Trigger for siren  on / off
volatile boolean sound1trigger = false;                  // Trigger for sound1  on / off
volatile boolean couplingTrigger = false;                // Trigger for trailer coupling  sound
volatile boolean uncouplingTrigger = false;              // Trigger for trailer uncoupling  sound
volatile boolean indicatorSoundOn = false;               // active, if indicator bulb is on

// Sound latches
volatile boolean hornLatch = false;                      // Horn latch bit
volatile boolean sirenLatch = false;                     // Siren latch bit

// Sound volumes
volatile uint16_t throttleDependentVolume = 0;           // engine volume according to throttle position
volatile uint16_t throttleDependentRevVolume = 0;        // engine rev volume according to throttle position
volatile uint16_t rpmDependentJakeBrakeVolume = 0;       // Engine rpm dependent jake brake volume
volatile uint16_t throttleDependentKnockVolume = 0;      // engine Diesel knock volume according to throttle position
volatile uint16_t throttleDependentTurboVolume = 0;      // turbo volume according to rpm
volatile uint16_t throttleDependentFanVolume = 0;        // cooling fan volume according to rpm
volatile uint16_t throttleDependentChargerVolume = 0;    // cooling fan volume according to rpm
volatile uint16_t throttleDependentWastegateVolume = 0;  // wastegate volume according to rpm
volatile int16_t masterVolume = 100;                     // Master volume percentage
volatile uint8_t dacOffset = 0;  // 128, but needs to be ramped up slowly to prevent popping noise, if switched on

// Throttle
int16_t currentThrottle = 0;                             // 0 - 500 (Throttle trigger input)
int16_t currentThrottleFaded = 0;                        // faded throttle for volume calculations etc.

// Engine
const int16_t maxRpm = 500;                              // always 500
const int16_t minRpm = 0;                                // always 0
int32_t currentRpm = 0;                                  // 0 - 500 (signed required!)
volatile uint8_t engineState = 0;                        // 0 = off, 1 = starting, 2 = running, 3 = stopping
int16_t engineLoad = 0;                                  // 0 - 500
volatile uint16_t engineSampleRate = 0;                  // Engine sample rate
int32_t speedLimit = maxRpm;                             // The speed limit, depending on selected virtual gear

// Clutch
boolean clutchDisengaged = true;                         // Active while clutch is disengaged

// Transmission
uint8_t selectedGear = 1;                                // The currently used gear of our shifting gearbox
uint8_t selectedAutomaticGear = 1;                       // The currently used gear of our automatic gearbox
boolean gearUpShiftingInProgress;                        // Active while shifting upwards
boolean gearDownShiftingInProgress;                      // Active while shifting downwards
boolean gearUpShiftingPulse;                             // Active, if shifting upwards begins
boolean gearDownShiftingPulse;                           // Active, if shifting downwards begins
volatile boolean neutralGear = false;                    // Transmission in neutral

// ESC
volatile boolean escIsBraking = false;                   // ESC is in a braking state
volatile boolean escIsDriving = false;                   // ESC is in a driving state
volatile boolean escInReverse = false;                   // ESC is driving or braking backwards
int8_t driveState = 0;                                   // for ESC state machine
int16_t escPulseMax;                                     // ESC calibration variables
int16_t escPulseMin;
int16_t escPulseMaxNeutral;
int16_t escPulseMinNeutral;
uint16_t currentSpeed = 0;                               // 0 - 500 (current ESC power)

// Our main tasks
TaskHandle_t Task1;

// Loop time (for debug)
uint16_t loopTime;

// Sampling intervals for interrupt timer (adjusted according to your sound file sampling rate)
uint32_t maxSampleInterval = 4000000 / sampleRate;
uint32_t minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;

// Interrupt timer for variable sample rate playback (engine sound)
hw_timer_t * variableTimer = NULL;
portMUX_TYPE variableTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t variableTimerTicks = maxSampleInterval;

// Interrupt timer for fixed sample rate playback (horn etc., playing in parallel with engine sound)
hw_timer_t * fixedTimer = NULL;
portMUX_TYPE fixedTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t fixedTimerTicks = maxSampleInterval;

//
// =======================================================================================================
// INTERRUPT FOR VARIABLE SPEED PLAYBACK (Engine sound, turbo sound)
// =======================================================================================================
//

void IRAM_ATTR variablePlaybackTimer() {

  static uint32_t attenuatorMillis;
  static uint32_t curEngineSample;              // Index of currently loaded engine sample
  static uint32_t curRevSample;                 // Index of currently loaded engine rev sample
  static uint32_t curTurboSample;               // Index of currently loaded turbo sample
  static uint32_t curFanSample;                 // Index of currently loaded fan sample
  static uint32_t curChargerSample;             // Index of currently loaded charger sample
  static uint32_t curStartSample;               // Index of currently loaded start sample
  static uint32_t curJakeBrakeSample;           // Index of currently loaded jake brake sample
  static uint32_t lastDieselKnockSample;        // Index of last Diesel knock sample
  static uint16_t attenuator;                   // Used for volume adjustment during engine switch off
  static uint16_t speedPercentage;              // slows the engine down during shutdown
  static int32_t a, a1, a2, a3, b, c, d, e;     // Input signals for mixer: a = engine, b = additional sound, c = turbo sound, d = fan sound, e = supercharger sound
  uint8_t a1Multi;                              // Volume multipliers

  //portENTER_CRITICAL_ISR(&variableTimerMux);

  switch (engineState) {

    case 0: // Engine off -----------------------------------------------------------------------
      variableTimerTicks = 4000000 / startSampleRate; // our fixed sampling rate
      timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

      a = 0; // volume = zero
      if (engineOn) {
        engineState = 1;
        engineStart = true;
      }
      break;

    case 1: // Engine start --------------------------------------------------------------------
      variableTimerTicks = 4000000 / startSampleRate; // our fixed sampling rate
      timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

      if (curStartSample < startSampleCount - 1) {
        a = (startSamples[curStartSample] * throttleDependentVolume / 100 * startVolumePercentage / 100);
        curStartSample ++;
      }
      else {
        curStartSample = 0;
        engineState = 2;
        engineStart = false;
        engineRunning = true;
        airBrakeTrigger = true;
      }
      break;

    case 2: // Engine running ------------------------------------------------------------------

      // Engine idle & revving sounds (mixed together according to engine rpm, new in v5.0)
      variableTimerTicks = engineSampleRate;  // our variable idle sampling rate!
      timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

      if (!engineJakeBraking) {
        if (curEngineSample < sampleCount - 1) {
          a1 = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100); // Idle sound
          a3 = 0;
          curEngineSample ++;

          // Optional rev sound, recorded at medium rpm. Note, that it needs to represent the same number of ignition cycles as the
          // idle sound. For example 4 or 8 for a V8 engine. It also needs to have about the same length. In order to adjust the length
          // or "revSampleCount", change the "Rate" setting in Audacity until it is about the same.
#ifdef REV_SOUND
          a2 = (revSamples[curRevSample] * throttleDependentRevVolume / 100 * revVolumePercentage / 100); // Rev sound
          if (curRevSample < revSampleCount) curRevSample ++;
#endif

          // Trigger throttle dependent Diesel ignition "knock" sound (played in the fixed sample rate interrupt)
          if (curEngineSample - lastDieselKnockSample > (sampleCount / dieselKnockInterval)) {
            dieselKnockTrigger = true;
            dieselKnockTriggerFirst = false;
            lastDieselKnockSample = curEngineSample;
          }
        }
        else {
          curEngineSample = 0;
          if (jakeBrakeRequest) engineJakeBraking = true;
#ifdef REV_SOUND
          curRevSample = 0;
#endif
          lastDieselKnockSample = 0;
          dieselKnockTrigger = true;
          dieselKnockTriggerFirst = true;
        }
        curJakeBrakeSample = 0;
      }
      else { // Jake brake sound ----
#ifdef JAKE_BRAKE_SOUND
        a3 = (jakeBrakeSamples[curJakeBrakeSample] * rpmDependentJakeBrakeVolume / 100 * jakeBrakeVolumePercentage / 100); // Jake brake sound
        a2 = 0;
        a1 = 0;
        if (curJakeBrakeSample < jakeBrakeSampleCount - 1) curJakeBrakeSample ++;
        else {
          curJakeBrakeSample = 0;
          if (!jakeBrakeRequest) engineJakeBraking = false;
        }

        curEngineSample = 0;
        curRevSample = 0;
#endif
      }

      // Engine sound mixer ----
#ifdef REV_SOUND
      // Mixing the idle and rev sounds together, according to engine rpm
      // Below the "revSwitchPoint" target, the idle volume precentage is 90%, then falling to 0% @ max. rpm.
      // The total of idle and rev volume percentage is always 100%

      if (currentRpm > revSwitchPoint) a1Multi = map(currentRpm, idleEndPoint, revSwitchPoint, 0, idleVolumeProportionPercentage);
      else a1Multi = idleVolumeProportionPercentage; // 90 - 100% proportion
      if (currentRpm > idleEndPoint) a1Multi = 0;

      a1 = a1 * a1Multi / 100; // Idle volume
      a2 = a2 * (100 - a1Multi) / 100; // Rev volume

      a = a1 + a2 + a3; // Idle and rev sounds mixed together
#else
      a = a1 + a3; // Idle sound only
#endif

      // Turbo sound ----------------------------------
      if (curTurboSample < turboSampleCount - 1) {
        c = (turboSamples[curTurboSample] * throttleDependentTurboVolume / 100 * turboVolumePercentage / 100);
        curTurboSample ++;
      }
      else {
        curTurboSample = 0;
      }

      // Fan sound -----------------------------------
      if (curFanSample < fanSampleCount - 1) {
        d = (fanSamples[curFanSample] * throttleDependentFanVolume / 100 * fanVolumePercentage / 100);
        curFanSample ++;
      }
      else {
        curFanSample = 0;
      }
#if defined GEARBOX_WHINING
      if (neutralGear) d = 0; // used for gearbox whining simulation, so not active in gearbox neutral
#endif

      // Supercharger sound --------------------------
      if (curChargerSample < chargerSampleCount - 1) {
        e = (chargerSamples[curChargerSample] * throttleDependentChargerVolume / 100 * chargerVolumePercentage / 100);
        curChargerSample ++;
      }
      else {
        curChargerSample = 0;
      }

      if (!engineOn) {
        speedPercentage = 100;
        attenuator = 1;
        engineState = 3;
        engineStop = true;
        engineRunning = false;
      }
      break;

    case 3: // Engine stop --------------------------------------------------------------------
      variableTimerTicks = 4000000 / sampleRate * speedPercentage / 100; // our fixed sampling rate
      timerAlarmWrite(variableTimer, variableTimerTicks, true); // // change timer ticks, autoreload true

      if (curEngineSample < sampleCount - 1) {
        a = (samples[curEngineSample] * throttleDependentVolume / 100 * idleVolumePercentage / 100 / attenuator);
        curEngineSample ++;
      }
      else {
        curEngineSample = 0;
      }

      // fade engine sound out
      if (millis() - attenuatorMillis > 100) { // Every 50ms
        attenuatorMillis = millis();
        attenuator ++; // attenuate volume
        speedPercentage += 20; // make it slower (10)
      }

      if (attenuator >= 50 || speedPercentage >= 500) { // 50 & 500
        a = 0;
        speedPercentage = 100;
        parkingBrakeTrigger = true;
        engineState = 4;
        engineStop = false;
      }
      break;

    case 4: // parking brake bleeding air sound after engine is off ----------------------------

      if (!parkingBrakeTrigger) {
        engineState = 0;
      }
      break;

  } // end of switch case

  // DAC output (groups a, b, c mixed together) ************************************************************************

  dacWrite(DAC1, constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5)) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC

  //portEXIT_CRITICAL_ISR(&variableTimerMux);
}

//
// =======================================================================================================
// INTERRUPT FOR FIXED SPEED PLAYBACK (Horn etc., played in parallel with engine sound)
// =======================================================================================================
//

void IRAM_ATTR fixedPlaybackTimer() {

  static uint32_t curHornSample;                           // Index of currently loaded horn sample
  static uint32_t curSirenSample;                          // Index of currently loaded siren sample
  static uint32_t curSound1Sample;                         // Index of currently loaded sound1 sample
  static uint32_t curReversingSample;                      // Index of currently loaded reversing beep sample
  static uint32_t curIndicatorSample;                      // Index of currently loaded indicator tick sample
  static uint32_t curWastegateSample;                      // Index of currently loaded wastegate sample
  static uint32_t curBrakeSample;                          // Index of currently loaded brake sound sample
  static uint32_t curParkingBrakeSample;                   // Index of currently loaded brake sound sample
  static uint32_t curShiftingSample;                       // Index of currently loaded shifting sample
  static uint32_t curDieselKnockSample;                    // Index of currently loaded Diesel knock sample
  static uint32_t curCouplingSample;                       // Index of currently loaded trailer coupling sample
  static uint32_t curUncouplingSample;                     // Index of currently loaded trailer uncoupling sample
  static int32_t a, a1, a2;                                // Input signals "a" for mixer
  static int32_t b, b0, b1, b2, b3, b4, b5, b6, b7, b8, b9;// Input signals "b" for mixer
  static boolean knockSilent;                              // This knock will be more silent
  static uint8_t curKnockCylinder;                         // Index of currently ignited zylinder

  //portENTER_CRITICAL_ISR(&fixedTimerMux);

  // Group "a" (horn & siren) ******************************************************************

  if (hornTrigger || hornLatch) {
    fixedTimerTicks = 4000000 / hornSampleRate; // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curHornSample < hornSampleCount - 1) {
      a1 =  (hornSamples[curHornSample] * hornVolumePercentage / 100);
      curHornSample ++;
#ifdef HORN_LOOP // Optional "endless loop" (points to be defined manually in horn file)
      if (hornTrigger && curHornSample == hornLoopEnd) curHornSample = hornLoopBegin; // Loop, if trigger still present
#endif
    }
    else { // End of sample
      curHornSample = 0;
      a1 = 0;
      hornLatch = false;
    }
  }

  if (sirenTrigger || sirenLatch) {
    fixedTimerTicks = 4000000 / sirenSampleRate; // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curSirenSample < sirenSampleCount - 1) {
      a2 = (sirenSamples[curSirenSample] * sirenVolumePercentage / 100);
      curSirenSample ++;
#ifdef SIREN_LOOP // Optional "endless loop" (points to be defined manually in siren file)
      if (sirenTrigger && curSirenSample == sirenLoopEnd) curSirenSample = sirenLoopBegin; // Loop, if trigger still present
#endif
    }
    else { // End of sample
      curSirenSample = 0;
      a2 = 0;
      sirenLatch = false;
    }
  }
  // if (curSirenSample > 10 && curSirenSample < 500) cannonFlash = true; // Tank cannon flash triggering in TRACKED_MODE
  // else cannonFlash = false;

  // Group "b" (other sounds) **********************************************************************

  // Sound 1 "b0" ----
  if (sound1trigger) {
    fixedTimerTicks = 4000000 / sound1SampleRate; // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curSound1Sample < sound1SampleCount - 1) {
      b0 = (sound1Samples[curSound1Sample] * sound1VolumePercentage / 100);
      curSound1Sample ++;
    }
    else {
      sound1trigger = false;
    }
  }
  else {
    curSound1Sample = 0; // ensure, next sound will start @ first sample
    b0 = 0;
  }

  // Reversing beep sound "b1" ----
  if (engineRunning && escInReverse) {
    fixedTimerTicks = 4000000 / reversingSampleRate; // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curReversingSample < reversingSampleCount - 1) {
      b1 = (reversingSamples[curReversingSample] * reversingVolumePercentage / 100);
      curReversingSample ++;
    }
    else {
      curReversingSample = 0;
    }
  }
  else {
    curReversingSample = 0; // ensure, next sound will start @ first sample
    b1 = 0;
  }

  // Indicator tick sound "b2" ----------------------------------------------------------------------
  if (indicatorSoundOn) {
    fixedTimerTicks = 4000000 / indicatorSampleRate; // our fixed sampling rate
    timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // // change timer ticks, autoreload true

    if (curIndicatorSample < indicatorSampleCount - 1) {
      b2 = (indicatorSamples[curIndicatorSample] * indicatorVolumePercentage / 100);
      curIndicatorSample ++;
    }
    else {
      indicatorSoundOn = false;
    }
  }
  else {
    curIndicatorSample = 0; // ensure, next sound will start @ first sample
    b2 = 0;
  }

  // Wastegate (blowoff) sound, triggered after rapid throttle drop -----------------------------------
  if (wastegateTrigger) {
    if (curWastegateSample < wastegateSampleCount - 1) {
      b3 = (wastegateSamples[curWastegateSample] * throttleDependentWastegateVolume / 100 * wastegateVolumePercentage / 100);
      curWastegateSample ++;
    }
    else {
      wastegateTrigger = false;
    }
  }
  else {
    b3 = 0;
    curWastegateSample = 0; // ensure, next sound will start @ first sample
  }

  // Air brake release sound, triggered after stop -----------------------------------------------
  if (airBrakeTrigger) {
    if (curBrakeSample < brakeSampleCount - 1) {
      b4 = (brakeSamples[curBrakeSample] * brakeVolumePercentage / 100);
      curBrakeSample ++;
    }
    else {
      airBrakeTrigger = false;
    }
  }
  else {
    b4 = 0;
    curBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Air parking brake attaching sound, triggered after engine off --------------------------------
  if (parkingBrakeTrigger) {
    if (curParkingBrakeSample < parkingBrakeSampleCount - 1) {
      b5 = (parkingBrakeSamples[curParkingBrakeSample] * parkingBrakeVolumePercentage / 100);
      curParkingBrakeSample ++;
    }
    else {
      parkingBrakeTrigger = false;
    }
  }
  else {
    b5 = 0;
    curParkingBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Pneumatic gear shifting sound, triggered while shifting the TAMIYA 3 speed transmission ------
  if (shiftingTrigger && engineRunning  && !automatic && !doubleClutch) {
    if (curShiftingSample < shiftingSampleCount - 1) {
      b6 = (shiftingSamples[curShiftingSample] * shiftingVolumePercentage / 100);
      curShiftingSample ++;
    }
    else {
      shiftingTrigger = false;
    }
  }
  else {
    b6 = 0;
    curShiftingSample = 0; // ensure, next sound will start @ first sample
  }

  // Diesel ignition "knock" played in fixed sample rate section, because we don't want changing pitch! ------
  if (dieselKnockTriggerFirst) {
    dieselKnockTriggerFirst = false;
    curKnockCylinder = 0;
  }

  if (dieselKnockTrigger) {
    dieselKnockTrigger = false;
    curKnockCylinder ++; // Count ignition sequence
    curDieselKnockSample = 0;
  }

#ifdef V8 // (former ADAPTIVE_KNOCK_VOLUME, rename it in your config file!)
  // Ford or Scania V8 ignition sequence: 1 - 5 - 4 - 2* - 6 - 3 - 7 - 8* (* = louder knock pulses, because 2nd exhaust in same manifold after 90°)
  if (curKnockCylinder == 4 || curKnockCylinder == 8) knockSilent = false;
  else knockSilent = true;
#endif

#ifdef V2
  // V2 engine: 1st and 2nd knock pulses (of 4) will be louder
  if (curKnockCylinder == 1 || curKnockCylinder == 2) knockSilent = false;
  else knockSilent = true;
#endif

#ifdef R6
  // R6 inline 6 engine: 6th knock pulse (of 6) will be louder
  if (curKnockCylinder == 6) knockSilent = false;
  else knockSilent = true;
#endif

  if (curDieselKnockSample < knockSampleCount && throttleDependentKnockVolume != 0) {
    b7 = (knockSamples[curDieselKnockSample] * dieselKnockVolumePercentage / 100 * throttleDependentKnockVolume / 100);
    curDieselKnockSample ++;
    if (knockSilent) b7 = b7 * dieselKnockAdaptiveVolumePercentage / 100; // changing knock volume according to engine type and cylinder!
  }

  // Trailer coupling sound, triggered by switch -----------------------------------------------
#ifdef COUPLING_SOUND
  if (couplingTrigger) {
    if (curCouplingSample < couplingSampleCount - 1) {
      b8 = (couplingSamples[curCouplingSample] * couplingVolumePercentage / 100);
      curCouplingSample ++;
    }
    else {
      couplingTrigger = false;
    }
  }
  else {
    b8 = 0;
    curCouplingSample = 0; // ensure, next sound will start @ first sample
  }

  // Trailer uncoupling sound, triggered by switch -----------------------------------------------
  if (uncouplingTrigger) {
    if (curUncouplingSample < uncouplingSampleCount - 1) {
      b9 = (uncouplingSamples[curUncouplingSample] * couplingVolumePercentage / 100);
      curUncouplingSample ++;
    }
    else {
      uncouplingTrigger = false;
    }
  }
  else {
    b9 = 0;
    curUncouplingSample = 0; // ensure, next sound will start @ first sample
  }
#endif

  // Mixing sounds together ----
  a = a1 + a2; // Horn & siren
  b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7 + b8 + b9; // Other sounds

  // DAC output (groups a + b mixed together) ****************************************************************************

  dacWrite(DAC2, constrain(((a * 8 / 10) + (b * 2 / 10)) * masterVolume / 100 + dacOffset, 0, 255)); // Mix signals, add 128 offset, write result to DAC

  //portEXIT_CRITICAL_ISR(&fixedTimerMux);
}

//
// =======================================================================================================
// PWM SIGNAL READ INTERRUPT
// =======================================================================================================
//

// Reference https://esp-idf.readthedocs.io/en/v1.0/api/rmt.html
static void IRAM_ATTR rmt_isr_handler(void* arg) {

  uint32_t intr_st = RMT.int_st.val;

  uint8_t i;
  for (i = 0; i < PWM_CHANNELS_NUM; i++) {
    uint8_t channel = PWM_CHANNELS[i];
    uint32_t channel_mask = BIT(channel * 3 + 1);

    if (!(intr_st & channel_mask)) continue;

    RMT.conf_ch[channel].conf1.rx_en = 0;
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
    volatile rmt_item32_t* item = RMTMEM.chan[channel].data32;
    if (item) {
      pulseWidthRaw[i + 1] = item->duration0;
    }

    RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
    RMT.conf_ch[channel].conf1.rx_en = 1;

    //clear RMT interrupt status.
    RMT.int_clr.val = channel_mask;
  }
}

// =======================================================================================================
// mcpwm SETUP (1x during startup)
// =======================================================================================================
//
// See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html#configure

void setupMcpwm() {
  // 1. set our servo output pins
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, STEERING_PIN);    //Set steering as PWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, SHIFTING_PIN);    //Set shifting as PWM0B
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, COUPLER_PIN);    //Set coupling as PWM1A

  // 2. configure MCPWM parameters
  mcpwm_config_t pwm_config;
  pwm_config.frequency = SERVO_FREQUENCY;     //frequency usually = 50Hz, some servos may run smoother @ 100Hz
  pwm_config.cmpr_a = 0;                      //duty cycle of PWMxa = 0
  pwm_config.cmpr_b = 0;                      //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;   // 0 = not inverted, 1 = inverted

  // 3. configure channels with settings above
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B
}

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void Task1code(void *pvParameters);
void readPwmSignals();
void processRawChannels();
void failsafeRcSignals();

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setupWIFI() {
  Serial.print("WIFI initialization...");
  // WiFi.mode(WIFI_MODE_STA);
  // Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();
  WiFi.softAP("ZIL_131", "12345678");
  delay(100);
  Serial.println(" done!");
  
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("handling endpoint '/'");
    request->send(200, "text/plain", "OK\n");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("handling endpoint '/data'");
    request->send(200, "text/plain", "data\n");
  });

  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("handling endpoint '/test'");
    request->send(200, "text/plain", "hej! to ja! zil 131\n");
  });

  server.on("/json", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("handling endpoint '/json'");
    
    AsyncJsonResponse *response = new AsyncJsonResponse();
    response->addHeader("Server","ESP Async Web Server");
    JsonObject& root = response->getRoot();
    root["heap"] = ESP.getFreeHeap();
    root["ssid"] = WiFi.SSID();
    response->setLength();
    request->send(response);
  });

  server.onNotFound(notFound);

  server.begin();
}


void setup() {
  // Watchdog timers need to be disabled, if task 1 is running without delay(1)
  disableCore0WDT();
  disableCore1WDT();

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the PWM variable.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xPwmSemaphore == NULL )  // Check to confirm that the PWM Semaphore has not already been created.
  {
    xPwmSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage variable access
    if ( ( xPwmSemaphore ) != NULL )
      xSemaphoreGive( ( xPwmSemaphore ) );  // Make the PWM variable available for use, by "Giving" the Semaphore.
  }

  // Set pin modes
  for (uint8_t i = 0; i < PWM_CHANNELS_NUM; i++) {
    pinMode(PWM_PINS[i], INPUT_PULLDOWN);
  }

  pinMode(PPM_PIN, INPUT_PULLDOWN);
  pinMode(COMMAND_RX, INPUT_PULLDOWN);

  // LED & shaker motor setup (note, that we only have timers from 0 - 15)
  headLight.begin(HEADLIGHT_PIN, 1, 20000); // Timer 1, 20kHz
  tailLight.begin(TAILLIGHT_PIN, 2, 20000); // Timer 2, 20kHz
  // indicatorL.begin(INDICATOR_LEFT_PIN, 3, 20000); // Timer 3, 20kHz
  // indicatorR.begin(INDICATOR_RIGHT_PIN, 4, 20000); // Timer 4, 20kHz
  // fogLight.begin(FOGLIGHT_PIN, 5, 20000); // Timer 5, 20kHz
  reversingLight.begin(REVERSING_LIGHT_PIN, 6, 20000); // Timer 6, 20kHz
  roofLight.begin(ROOFLIGHT_PIN, 7, 20000); // Timer 7, 20kHz

  escOut.begin(ESC_OUT_PIN, 15, 50, 16); // Timer 15, 50Hz, 16bit <-- ESC running @ 50Hz

  // Serial setup
  Serial.begin(115200); // USB serial (for DEBUG)
  
  // Wifi setup
  setupWIFI();

  // PWM ----
  if (MAX_RPM_PERCENTAGE > maxPwmRpmPercentage) MAX_RPM_PERCENTAGE = maxPwmRpmPercentage; // Limit RPM range
  // New: PWM read setup, using rmt. Thanks to croky-b
  uint8_t i;
  rmt_config_t rmt_channels[PWM_CHANNELS_NUM] = {};

  for (i = 0; i < PWM_CHANNELS_NUM; i++) {
    rmt_channels[i].channel = (rmt_channel_t) PWM_CHANNELS[i];
    rmt_channels[i].gpio_num = (gpio_num_t) PWM_PINS[i];
    rmt_channels[i].clk_div = RMT_RX_CLK_DIV;
    rmt_channels[i].mem_block_num = 1;
    rmt_channels[i].rmt_mode = RMT_MODE_RX;
    rmt_channels[i].rx_config.filter_en = true;
    rmt_channels[i].rx_config.filter_ticks_thresh = 100; // Pulses shorter than this will be filtered out
    rmt_channels[i].rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;

    rmt_config(&rmt_channels[i]);
    rmt_set_rx_intr_en(rmt_channels[i].channel, true);
    rmt_rx_start(rmt_channels[i].channel, 1);
  }

  rmt_isr_register(rmt_isr_handler, NULL, 0, NULL); // This is our interrupt

  // -----------------------------------------------------------

  // Refresh sample intervals (important, because MAX_RPM_PERCENTAGE was probably changed above)
  maxSampleInterval = 4000000 / sampleRate;
  minSampleInterval = 4000000 / sampleRate * 100 / MAX_RPM_PERCENTAGE;

  // Time
  timelast = micros();
  timelastloop = timelast;


  // Task 1 setup (running on core 0)
  TaskHandle_t Task1;
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    100000,       /* Stack size of task (10000) */
    NULL,        /* parameter of the task */
    1,           /* priority of the task (1 = low, 3 = medium, 5 = highest)*/
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */

  // Interrupt timer for variable sample rate playback
  variableTimer = timerBegin(0, 20, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(variableTimer, &variablePlaybackTimer, true); // edge (not level) triggered
  timerAlarmWrite(variableTimer, variableTimerTicks, true); // autoreload true
  timerAlarmEnable(variableTimer); // enable

  // Interrupt timer for fixed sample rate playback
  fixedTimer = timerBegin(1, 20, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 20 -> 250 ns = 0.25 us, countUp
  timerAttachInterrupt(fixedTimer, &fixedPlaybackTimer, true); // edge (not level) triggered
  timerAlarmWrite(fixedTimer, fixedTimerTicks, true); // autoreload true
  timerAlarmEnable(fixedTimer); // enable

  // wait for RC receiver to initialize
  while (millis() <= 1000);

  readPwmSignals();


  // Calculate RC input signal ranges for all channels
  for (uint8_t i = 1; i < PULSE_ARRAY_SIZE; i++) {
    pulseZero[i] = 1500; // 1500 is the center position. Auto centering is now done in "processRawChannels()"

    // Input signals
    pulseMaxNeutral[i] = pulseZero[i] + pulseNeutral;
    pulseMinNeutral[i] = pulseZero[i] - pulseNeutral;
    pulseMax[i] = pulseZero[i] + pulseSpan;
    pulseMin[i] = pulseZero[i] - pulseSpan;
    pulseMaxLimit[i] = pulseZero[i] + pulseLimit;
    pulseMinLimit[i] = pulseZero[i] - pulseLimit;
  }

  // ESC output calibration
  escPulseMaxNeutral = pulseZero[3] + escTakeoffPunch; //Additional takeoff punch around zero
  escPulseMinNeutral = pulseZero[3] - escTakeoffPunch;

  escPulseMax = pulseZero[3] + escPulseSpan;
  escPulseMin = pulseZero[3] - escPulseSpan + escReversePlus; //Additional power for ESC with slow reverse
}

//
// =======================================================================================================
// DAC OFFSET FADER
// =======================================================================================================
//

static unsigned long dacOffsetMicros;
boolean dacInit;

void dacOffsetFade() {
  if (!dacInit) {
    if (micros() - dacOffsetMicros > 100) { // Every 0.1ms
      dacOffsetMicros = micros();
      dacOffset ++; // fade DAC offset slowly to prevent it from popping, if ESP32 powered up after amplifier
      if (dacOffset == 128) dacInit = true;
    }
  }
}

//
// =======================================================================================================
// READ PWM RC SIGNALS (plug in your channels according to order in "channelSetup.h")
// =======================================================================================================
//

void readPwmSignals() {

  // measure RC signal pulsewidth:
  // nothing is done here, the PWM signals are now read, using the
  // "static void IRAM_ATTR rmt_isr_handler(void* arg)" interrupt function

  // NOTE: There is no channel mapping in this mode! Just plug in the wires in the order as defined in "channelsetup.h"
  // for example: sound controller channel 2 (GEARBOX) connects to receiver channel 6

  // Normalize, auto zero and reverse channels
  processRawChannels();

  // Failsafe for RC signals
  failSafe = (pulseWidthRaw[3] < 500 || pulseWidthRaw[3] > 2500);
  failsafeRcSignals();
}

//
// =======================================================================================================
// PROZESS CHANNELS (Normalize, auto zero and reverse)
// =======================================================================================================
//

void processRawChannels() {

  if (xSemaphoreTake( xPwmSemaphore, portMAX_DELAY )) {
    for (int i = 1; i < PULSE_ARRAY_SIZE; i++) {

      // Take channel raw data, reverse them, if required and store them
      if (channelReversed[i]) pulseWidth[i] = map(pulseWidthRaw[i], 0, 3000, 3000, 0); // Reversed
      else pulseWidth[i] = pulseWidthRaw[i]; // Not reversed

      // Auto zero offset adjustment (only within certain absolute range)
      if (channelAutoZero[i] && !autoZeroDone && pulseWidth[i] > pulseMinValid && pulseWidth[i] < pulseMaxValid) pulseOffset[i] = 1500 - pulseWidth[i];

      // Compensate pulsewidth with auto zero offset
      pulseWidth[i] += pulseOffset[i];
      if (!autoZeroDone) { // Print offsets, if switching on the controller
        Serial.println(i);
        Serial.println(pulseOffset[i]);
      }

      // Set auto zero done flag
      if (i == PULSE_ARRAY_SIZE - 1) autoZeroDone = true;
      xSemaphoreGive( xPwmSemaphore );
    }
  }
  else {
    xSemaphoreGive( xPwmSemaphore ); // Free or "Give" the semaphore for others, if not required!
  }

  // Print input signal debug infos

#ifdef CHANNEL_DEBUG // can slow down the playback loop!
  static unsigned long printChannelMillis;
  if (millis() - printChannelMillis > 1000) { // Every 1000ms
    printChannelMillis = millis();

    Serial.println("Channels [microseconds]:");
    Serial.println(pulseWidth[1]);
    Serial.println(pulseWidth[2]);
    Serial.println(pulseWidth[3]);
    Serial.println(pulseWidth[4]);
    Serial.println(pulseWidth[5]);
    Serial.println(pulseWidth[6]);
    Serial.println(pulseWidth[7]);
    Serial.println(pulseWidth[8]);
    Serial.println(pulseWidth[9]);
    Serial.println(pulseWidth[10]);
    Serial.println(pulseWidth[11]);
    Serial.println(pulseWidth[12]);
    Serial.println(pulseWidth[13]);
    Serial.println(pulseWidth[14]);
    Serial.println(pulseWidth[15]);
    Serial.println(pulseWidth[16]);
    Serial.println(currentThrottle);
    Serial.println("States:");
    Serial.println(mode1);
    Serial.println(mode2);
    Serial.println(momentary1);
    Serial.println(hazard);
    Serial.println(left);
    Serial.println(right);
    Serial.println("Failsafe:");
    // Serial.println(SBUSfailSafe);
    // Serial.println(SBUSlostFrame);
    Serial.println(failSafe);
    Serial.println(MAX_RPM_PERCENTAGE);
    Serial.println("");
  }
#endif
}

//
// =======================================================================================================
// RC SIGNAL FAILSAFE POSITIONS (if serial signal lost)
// =======================================================================================================
//

void failsafeRcSignals() {

  // Failsafe actions --------
  if (failSafe) pulseWidth[3] = pulseZero[3]; // Throttle to zero position!
}

//
// =======================================================================================================
// MAP PULSEWIDTH TO THROTTLE CH3
// =======================================================================================================
//

void mapThrottle() {

  // Input is around 1000 - 2000us, output 0-500 for forward and backwards
  // check if the pulsewidth looks like a servo pulse
  if (pulseWidth[3] > pulseMinLimit[3] && pulseWidth[3] < pulseMaxLimit[3]) {
    if (pulseWidth[3] < pulseMin[3]) pulseWidth[3] = pulseMin[3]; // Constrain the value
    if (pulseWidth[3] > pulseMax[3]) pulseWidth[3] = pulseMax[3];

    // calculate a throttle value from the pulsewidth signal
    if (pulseWidth[3] > pulseMaxNeutral[3]) {
      currentThrottle = map(pulseWidth[3], pulseMaxNeutral[3], pulseMax[3], 0, 500);
    }
    else if (pulseWidth[3] < pulseMinNeutral[3]) {
      currentThrottle = map(pulseWidth[3], pulseMinNeutral[3], pulseMin[3], 0, 500);
    }
    else {
      currentThrottle = 0;
    }
  }

  // Auto throttle --------------------------------------------------------------------------

  // Auto throttle while gear shifting (synchronizing the Tamiya 3 speed gearbox)
  if (!escIsBraking && escIsDriving && shiftingAutoThrottle) {
    if (gearUpShiftingInProgress) currentThrottle = 0; // No throttle
    if (gearDownShiftingInProgress) currentThrottle = 500; // Full throttle
    currentThrottle = constrain (currentThrottle, 0, 500);
  }

  // Volume calculations --------------------------------------------------------------------------

  // As a base for some calculations below, fade the current throttle to make it more natural
  static unsigned long throttleFaderMicros;
  if (micros() - throttleFaderMicros > 500) { // Every 0.5ms
    throttleFaderMicros = micros();

    if (currentThrottleFaded < currentThrottle && currentThrottleFaded < 499) currentThrottleFaded += 2;
    if (currentThrottleFaded > currentThrottle && currentThrottleFaded > 2) currentThrottleFaded -= 2;
    //Serial.println(currentThrottleFaded);
  }

  // Calculate throttle dependent engine idle volume
  if (!escIsBraking && engineRunning) throttleDependentVolume = map(currentThrottleFaded, 0, 500, engineIdleVolumePercentage, fullThrottleVolumePercentage);
  else throttleDependentVolume = engineIdleVolumePercentage;

  // Calculate throttle dependent engine rev volume
  if (!escIsBraking && engineRunning) throttleDependentRevVolume = map(currentThrottleFaded, 0, 500, engineRevVolumePercentage, fullThrottleVolumePercentage);
  else throttleDependentRevVolume = engineRevVolumePercentage;

  // Calculate engine rpm dependent jake brake volume
  if (engineRunning) rpmDependentJakeBrakeVolume = map(currentRpm, 0, 500, jakeBrakeIdleVolumePercentage, 100);
  else rpmDependentJakeBrakeVolume = jakeBrakeIdleVolumePercentage;

  // Calculate throttle dependent Diesel knock volume
  if (!escIsBraking && engineRunning && (currentThrottleFaded > dieselKnockStartPoint)) throttleDependentKnockVolume = map(currentThrottleFaded, dieselKnockStartPoint, 500, dieselKnockIdleVolumePercentage, 100);
  else throttleDependentKnockVolume = dieselKnockIdleVolumePercentage;

  // Calculate engine rpm dependent turbo volume
  if (engineRunning) throttleDependentTurboVolume = map(currentRpm, 0, 500, turboIdleVolumePercentage, 100);
  else throttleDependentTurboVolume = turboIdleVolumePercentage;

  // Calculate engine rpm dependent cooling fan volume
  if (engineRunning && (currentRpm > fanStartPoint)) throttleDependentFanVolume = map(currentRpm, fanStartPoint, 500, fanIdleVolumePercentage, 100);
  else throttleDependentFanVolume = fanIdleVolumePercentage;

  // Calculate throttle dependent supercharger volume
  if (!escIsBraking && engineRunning && (currentRpm > chargerStartPoint)) throttleDependentChargerVolume = map(currentThrottleFaded, chargerStartPoint, 500, chargerIdleVolumePercentage, 100);
  else throttleDependentChargerVolume = chargerIdleVolumePercentage;

  // Calculate engine rpm dependent wastegate volume
  if (engineRunning) throttleDependentWastegateVolume = map(currentRpm, 0, 500, wastegateIdleVolumePercentage, 100);
  else throttleDependentWastegateVolume = wastegateIdleVolumePercentage;


  // Calculate engine load (used for torque converter slip simulation)
  engineLoad = currentThrottle - currentRpm;
  // if (engineLoad < 0 || escIsBraking) engineLoad = 0; // Range is 0 - 500
  if (engineLoad < 0) engineLoad = 0; // Range is 0 - 500
  if (engineLoad > 180) engineLoad = 180;
}

//
// =======================================================================================================
// ENGINE MASS SIMULATION
// =======================================================================================================
//

void engineMassSimulation() {

  static int32_t  targetRpm = 0;        // The engine RPM target
  static int32_t  lastThrottle;
  uint16_t converterSlip;
  static unsigned long throtMillis;
  static unsigned long printMillis;
  static unsigned long wastegateMillis;
  uint8_t timeBase;
  timeBase = 2;

  if (millis() - throtMillis > timeBase) { // Every 2 or 6ms
    throtMillis = millis();

    if (currentThrottle > 500) currentThrottle = 500;

    // Virtual clutch **********************************************************************************
    if ((currentSpeed < clutchEngagingPoint && currentRpm < maxClutchSlippingRpm) || gearUpShiftingInProgress || gearDownShiftingInProgress || neutralGear || currentRpm < 200) {
      clutchDisengaged = true;
    }
    else {
      clutchDisengaged = false;
    }


    // Transmissions ***********************************************************************************

    // automatic transmission ----
    if (automatic) {
      // Torque converter slip calculation
      if (selectedAutomaticGear < 2) converterSlip = engineLoad * 2; // more slip in first and reverse gear
      else converterSlip = engineLoad;

      if (!neutralGear) targetRpm = currentSpeed * gearRatio[selectedAutomaticGear] / 10 + converterSlip; // Compute engine RPM
      else targetRpm = reMap(curveLinear, currentThrottle);
    }
    else if (doubleClutch) {
      // double clutch transmission
      if (!neutralGear) targetRpm = currentSpeed * gearRatio[selectedAutomaticGear] / 10; // Compute engine RPM
      else targetRpm = reMap(curveLinear, currentThrottle);
    }
    else {
      // Manual transmission ----
      if (clutchDisengaged) { // Clutch disengaged: Engine revving allowed
        targetRpm = reMap(curveLinear, currentThrottle);
      }
      else { // Clutch engaged: Engine rpm synchronized with ESC power (speed)
        targetRpm = reMap(curveLinear, currentSpeed);
      }
    }

    // Engine RPM **************************************************************************************

    if (escIsBraking && currentSpeed < clutchEngagingPoint) targetRpm = 0; // keep engine @idle rpm, if braking at very low speed
    if (targetRpm > 500) targetRpm = 500;

    // [GN] Slowing down when trhottle is neutral
    if (currentThrottle >= 0 && currentThrottle < 10 && currentRpm > 0 && (driveState == 1 || driveState == 3)) {
      // Serial.print("target rpm: ");
      // Serial.println(targetRpm);
      // Serial.print("current rpm: ");
      // Serial.println(currentRpm);
      // currentRpm = targetRpm - 10;
    }


    // Accelerate engine
    if (targetRpm > (currentRpm + acc) && (currentRpm + acc) < maxRpm && engineState == 2 && engineRunning) {
      if (!airBrakeTrigger) { // No acceleration, if brake release noise still playing
        if (!gearDownShiftingInProgress) currentRpm += acc;
        else currentRpm += acc / 2; // less aggressive rpm rise while downshifting
        if (currentRpm > maxRpm) currentRpm = maxRpm;
      }
    }

    // Decelerate engine
    if (targetRpm < currentRpm /*&& currentThrottle > 0*/) {
      currentRpm -= dec;
      if (currentRpm < minRpm) currentRpm = minRpm;
    }

    // [GN ]Slow down when throttle in neutral
    // if (currentThrottle == 0 && (driveState == 1 || driveState == 3)) {
    //   currentRpm -= (dec*3);
    //   if (currentRpm < minRpm) currentRpm = minRpm;
    // }

    // Speed (sample rate) output
    engineSampleRate = map(currentRpm, minRpm, maxRpm, maxSampleInterval, minSampleInterval); // Idle
  }

  // Prevent Wastegate from being triggered while downshifting
  if (gearDownShiftingInProgress) wastegateMillis = millis();

  // Trigger Wastegate, if throttle rapidly dropped
  if (lastThrottle - currentThrottle > 70 && !escIsBraking && millis() - wastegateMillis > 1000) {
    wastegateMillis = millis();
    wastegateTrigger = true;
  }
  lastThrottle = currentThrottle;
}

//
// =======================================================================================================
// SWITCH ENGINE ON OR OFF (for automatic mode)
// =======================================================================================================
//

void engineOnOff() {

  static unsigned long pulseDelayMillis;
  static unsigned long idleDelayMillis;

  // Engine automatically switched on or off depending on throttle position and 15s delay timne
  if (currentThrottle > 80 || driveState != 0) idleDelayMillis = millis(); // reset delay timer, if throttle not in neutral


#ifdef AUTO_ENGINE_ON_OFF
  if (millis() - idleDelayMillis > 15000) {
    engineOn = false; // after delay, switch engine off
  }
#endif

  // Engine start detection
  if (currentThrottle > 100 && !airBrakeTrigger) {
    #ifdef AUTO_ENGINE_ON_OFF
    engineOn = true;
    #endif
    // lightsOn = true;
  }
}


//
// =======================================================================================================
// SHAKER (simulates engine vibrations)
// =======================================================================================================
//

void shaker() {
  int32_t shakerRpm;

  // Set desired shaker rpm
  if (engineRunning) shakerRpm = map(currentRpm, minRpm, maxRpm, shakerIdle, shakerFullThrottle);
  if (engineStart) shakerRpm = shakerStart;
  if (engineStop) shakerRpm = shakerStop;

  // Shaker on / off
  // if (engineRunning || engineStart || engineStop) shakerMotor.pwm(shakerRpm);
  // else shakerMotor.off();
}


//
// =======================================================================================================
// SIMULATED AUTOMATIC TRANSMISSION GEAR SELECTOR
// =======================================================================================================
//

void automaticGearSelector() {

  static unsigned long gearSelectorMillis;
  static unsigned long lastUpShiftingMillis;
  static unsigned long lastDownShiftingMillis;
  uint16_t downShiftPoint = 200;
  uint16_t upShiftPoint = 490;

  if (millis() - gearSelectorMillis > 100) { // Waiting for 100ms is very important. Otherwise gears are skipped!
    gearSelectorMillis = millis();

    // compute load dependent shift points (less throttle = less rpm before shifting up, kick down will shift back!)
    upShiftPoint = map(engineLoad, 0, 180, 390, 490); // 390, 490
    downShiftPoint = map(engineLoad, 0, 180, 150, 250); // 150, 250

    if (escInReverse) { // Reverse (only one gear)
      // Serial.println("REVERSE");
      selectedAutomaticGear = 0;
    }
    else { // Forward

      // Adaptive shift points
      if (millis() - lastDownShiftingMillis > 500 && currentRpm >= upShiftPoint && engineLoad < 5) { // 500ms locking timer!
        selectedAutomaticGear ++; // Upshifting (load maximum is important to prevent gears from oscillating!)
        lastUpShiftingMillis = millis();
        // Serial.println("upshifting");
      }
      if (millis() - lastUpShiftingMillis > 1000 && selectedAutomaticGear > 1 && (currentRpm <= downShiftPoint || engineLoad > 100)) { // 1000ms locking timer!
        selectedAutomaticGear --; // Downshifting incl. kickdown
        lastDownShiftingMillis = millis();
        // Serial.println("downshifting");
      }

      selectedAutomaticGear = constrain(selectedAutomaticGear, 1, NumberOfAutomaticGears);
    }

#ifdef AUTO_TRANS_DEBUG
    Serial.print("driveState:     ");
    Serial.println(driveState);
    Serial.print("Current Speed:  ");
    Serial.println(currentSpeed);
    Serial.print("Throttle:       ");
    Serial.println(currentThrottle);
    Serial.print("Gear:           ");
    Serial.println(selectedAutomaticGear);
    Serial.print("Engine Load:    ");
    Serial.println(engineLoad);
    // Serial.print("UpShiftPoint:   ");
    // Serial.println(upShiftPoint);
    Serial.print("RPM:            ");
    Serial.println(currentRpm);
    // Serial.print("DownShiftPoint: ");
    // Serial.println(downShiftPoint);
    Serial.println("===========");
#endif
  }
}


#define DRIVE_STATE_STAND_STILL 0
#define DRIVE_STATE_FORWARD 1
#define DRIVE_STATE_FORWARD_BRAKE 2
#define DRIVE_STATE_BACKWARD 3
#define DRIVE_STATE_BACKWARD_BRAKE 4


void newEngineSimulation() {
  static int32_t escPulseWidth = 1500;
  static uint32_t escSignal;
  static unsigned long escMillis;
  static unsigned long lastStateTime;
  static int8_t pulse; // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t escPulse; // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t driveRampRate;
  static int8_t driveRampGain;
  static int8_t brakeRampRate;
  uint8_t escRampTime;

  if (millis() - escMillis > escRampTime) { // About very 20 - 75ms
    escMillis = millis();

    // calulate throttle dependent brake & acceleration steps
    brakeRampRate = map (currentThrottle, 0, 500, 0, escBrakeSteps);
    driveRampRate = map (currentThrottle, 0, 500, 0, escAccelerationSteps);

    // Emergency ramp rates for falisafe
    if (failSafe) {
      brakeRampRate = escBrakeSteps;
      driveRampRate = escBrakeSteps;
    }

    // Comparators
    if (pulseWidth[3] > pulseMaxNeutral[3] && pulseWidth[3] < pulseMaxLimit[3]) pulse = 1; // 1 = Forward
    else if (pulseWidth[3] < pulseMinNeutral[3] && pulseWidth[3] > pulseMinLimit[3]) pulse = -1; // -1 = Backwards
    else pulse = 0; // 0 = Neutral

    if (escPulseWidth > pulseMaxNeutral[3] && escPulseWidth < pulseMaxLimit[3]) escPulse = 1; // 1 = Forward
    else if (escPulseWidth < pulseMinNeutral[3] && escPulseWidth > pulseMinLimit[3]) escPulse = -1; // -1 = Backwards
    else escPulse = 0; // 0 = Neutral

#ifdef ESC_DEBUG
    if (millis() - lastStateTime > 300) { // Print the data every 300ms
      lastStateTime = millis();
      Serial.println("===== ESC ======");
      // Serial.println(driveState);
      // Serial.println(pulse);
      Serial.print("escPulse: ");
      Serial.println(escPulse);
      Serial.println(escPulseMin);
      Serial.println(escPulseMax);
      // Serial.println(brakeRampRate);
      // Serial.println(currentRpm);
      // Serial.println(currentSpeed);
      // Serial.println(speedLimit);
      Serial.println("===============");
    }
#endif

    // Drive state state machine **********************************************************************************
    switch (driveState) {
      case DRIVE_STATE_STAND_STILL: // Standing still ---------------------------------------------------------------------
        escIsBraking = false;
        escInReverse = false;
        escIsDriving = false;
        escPulseWidth = pulseZero[3];  // ESC to neutral position

        if (pulse == 1 && engineRunning && !neutralGear) driveState = DRIVE_STATE_FORWARD; // Driving forward
        if (pulse == -1 && engineRunning && !neutralGear) driveState = DRIVE_STATE_BACKWARD; // Driving backwards
        break;

      case DRIVE_STATE_FORWARD: // Driving forward ---------------------------------------------------------------------
        escIsBraking = false;
        escInReverse = false;
        escIsDriving = true;
        if (escPulseWidth < pulseWidth[3] && currentSpeed < speedLimit) {
          if (escPulseWidth >= escPulseMaxNeutral) escPulseWidth += (driveRampRate * driveRampGain); //Faster
          else escPulseWidth = escPulseMaxNeutral; // Initial boost
        }
        if (escPulseWidth > pulseWidth[3] && escPulseWidth > pulseZero[3]) escPulseWidth -= (driveRampRate * driveRampGain); // Slower

        if (gearUpShiftingPulse && shiftingAutoThrottle) { // lowering RPM, if shifting up transmission
          gearUpShiftingPulse = false;
          escPulseWidth = constrain(escPulseWidth, pulseZero[3], pulseMax[3]);
        }
        if (gearDownShiftingPulse && shiftingAutoThrottle) { // increasing RPM, if shifting down transmission
          gearDownShiftingPulse = false;
          escPulseWidth = constrain(escPulseWidth, pulseZero[3], pulseMax[3]);
        }

        if (pulse == -1 && escPulse == 1) {
          driveState = DRIVE_STATE_STAND_STILL; // Change direction
          airBrakeTrigger = true;
        }
        if (pulse == 0 && escPulse == 0) driveState = DRIVE_STATE_STAND_STILL; // standing still
        break;

      case DRIVE_STATE_BACKWARD: // Driving backwards ---------------------------------------------------------------------
        escIsBraking = false;
        escInReverse = true;
        escIsDriving = true;
        if (escPulseWidth > pulseWidth[3] && currentSpeed < speedLimit) {
          if (escPulseWidth <= escPulseMinNeutral) escPulseWidth -= (driveRampRate * driveRampGain); //Faster
          else escPulseWidth = escPulseMinNeutral; // Initial boost
        }
        if (escPulseWidth < pulseWidth[3] && escPulseWidth < pulseZero[3]) escPulseWidth += (driveRampRate * driveRampGain); // Slower

        if (gearUpShiftingPulse && shiftingAutoThrottle) { // lowering RPM, if shifting up transmission
          gearUpShiftingPulse = false;
          escPulseWidth = constrain(escPulseWidth, pulseMin[3], pulseZero[3]);
        }
        if (gearDownShiftingPulse && shiftingAutoThrottle) { // increasing RPM, if shifting down transmission
          gearDownShiftingPulse = false;
          escPulseWidth = constrain(escPulseWidth, pulseMin[3], pulseZero[3]);
        }

        if (pulse == 1 && escPulse == -1) {
          driveState = DRIVE_STATE_STAND_STILL; // Change direction
          airBrakeTrigger = true;
        }

        if (pulse == 0 && escPulse == 0) driveState = DRIVE_STATE_STAND_STILL; // standing still
        break;
    } // End of state machine **********************************************************************************


    // Gain for drive ramp rate, depending on clutchEngagingPoint
    if (currentSpeed < clutchEngagingPoint) {
      if (!automatic  && !doubleClutch) driveRampGain = 2; // prevent clutch from slipping too much (2)
      else driveRampGain = 4; // Automatic transmission needs to catch immediately (4)
    }
    else driveRampGain = 1;


    // ESC control
    
    // if (escPulseMin > 0 && escPulseMax > 0) {
    //   escSignal = map(escPulseWidth, escPulseMin, escPulseMax, 3278, 6553); // 1 - 2ms (5 - 10% pulsewidth of 65534, not reversed)
    // } else {
    //   Serial.println("NOOOOOOO!!!!!!!");
    // }
    
    // escOut.pwm(escSignal);

    // Calculate a speed value from the pulsewidth signal (used as base for engine sound RPM while clutch is engaged)
    // if (escPulseWidth > pulseMaxNeutral[3]) {
    //   currentSpeed = map(escPulseWidth, pulseMaxNeutral[3], pulseMax[3], 0, 500);
    // }
    // else if (escPulseWidth < pulseMinNeutral[3]) {
    //   currentSpeed = map(escPulseWidth, pulseMinNeutral[3], pulseMin[3], 0, 500);
    // }
    // else currentSpeed = 0;

    // For now let's map speed directly with trottle (best solution for crawler)
    currentSpeed = currentThrottle;
  }
}


//
// =======================================================================================================
// LOOP TIME MEASUREMENT
// =======================================================================================================
//

unsigned long loopDuration() {
  static unsigned long timerOld;
  unsigned long loopTime;
  unsigned long timer = millis();
  loopTime = timer - timerOld;
  timerOld = timer;
  return loopTime;
}

bool headLightsOn = false;
bool roofLightsOn = false;

void checkButtons() {
  // CH 5 - engine on/off, horn
  if (pulseWidth[5] < 1200) {
    // Engine off
    engineOn = false;
    hornTrigger = false;
    hornLatch = false;
  } else if (pulseWidth[5] > 1200 && pulseWidth[5] <= 1700) {
    // Engine on
    engineOn = true;
    hornTrigger = false;
    hornLatch = false;
  } else if (pulseWidth[5] > 1700) {
    // horn
    engineOn = true;
    hornTrigger = true;
    hornLatch = true;
  }

  // CH 6 - lights
  if (pulseWidth[6] < 1200) {
    headLight.off();
    roofLight.off();

    headLightsOn = false;
    roofLightsOn = false;

  } else if (pulseWidth[6] > 1200 && pulseWidth[6] <= 1500) {
    headLight.pwm(80);
    roofLight.off();
    headLightsOn = true;
    roofLightsOn = false;
  } else if (pulseWidth[6] > 1500 && pulseWidth[6] <= 1700) {
    headLight.on();
    roofLight.off();
    headLightsOn = true;
    roofLightsOn = false;
  }
  else if (pulseWidth[6] > 1700) {
    headLight.on();
    roofLight.on();
    headLightsOn = true;
    roofLightsOn = true;
  }
}

void processReverseLight() {
  if (engineRunning && escInReverse) {
    // Reverse lights on
    reversingLight.on();
  } else {
    // Reverse lights off
    reversingLight.off();
  }
}

bool braking = false;
#define TAIL_LIGHT_BRIGHTNES 30
#define STOP_LIGHT_BRIGHTNES 80

void processStopLight() {
  static unsigned long lastStopLightMilis = millis();
  static int16_t lastThrottle = 0;

  if (engineRunning) {
    if (currentThrottle == 0) {
      braking = true;
    } else if (millis() - lastStopLightMilis >= 300) {
      int16_t throttleDifference = lastThrottle - currentThrottle;

      if (throttleDifference > 10) {
        braking = true;
      } else {
        braking = false;
      }

      lastThrottle = currentThrottle;
      lastStopLightMilis = millis();
    }
  } else {
    braking = false;
  }

  if (braking) {
    tailLight.pwm(STOP_LIGHT_BRIGHTNES);
  } else {
    if (headLightsOn) {
      tailLight.pwm(TAIL_LIGHT_BRIGHTNES);
    } else {
      tailLight.off();
    }
  }

  // Serial.print("is braking: ");
  // Serial.println(braking ? "yes" : "no");

  // Serial.print("current throttle: ");
  // Serial.print(currentThrottle);
  // Serial.print(" engineRunning: ");
  // Serial.print(engineRunning ? "yes" : "no");
  // Serial.print(" reverse: ");
  // Serial.println(escInReverse ? "yes" : "no");
}

//
// =======================================================================================================
// MAIN LOOP, RUNNING ON CORE 1
// =======================================================================================================
//

void loop() {

  // measure RC signals mark space ratio
  readPwmSignals();

  // Map pulsewidth to throttle
  mapThrottle();

  // Check buttons actions
  checkButtons();

  // Auto lights
  processReverseLight();
  processStopLight();

  // static unsigned long lastWifiPrint = millis();
  // if (millis() - lastWifiPrint > 1000) {
  //   Serial.print("WIFI address: ");
  //   Serial.println(WiFi.gatewayIP());
  //   lastWifiPrint = millis();
  // }
}

//
// =======================================================================================================
// 1st MAIN TASK, RUNNING ON CORE 0 (Interrupts are running on this core as well)
// =======================================================================================================
//

void Task1code(void *pvParameters) {
  for (;;) {

    // DAC offset fader
    dacOffsetFade();

    // Simulate engine mass, generate RPM signal
    engineMassSimulation();

    // Call gear selector
    if (automatic || doubleClutch) automaticGearSelector();

    newEngineSimulation();

    // measure loop time
    //loopTime = loopDuration(); // for debug only
  }
}
