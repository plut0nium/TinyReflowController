/******************************************************************************
 * TINY REFLOW CONTROLLER
 * 
 * This software is a firmware mod for the Tiny Reflow Controller v2.0
 * by Rocket Scream Electronics (www.rocketscream.com)
 * 
 * See README for more information
 * 
 * 
 * DISCLAIMER
 * Dealing with high voltage is a very dangerous act! Please make sure you know
 * what you are dealing with and have proper knowledge before hand. Your use of
 * any information or materials on this Tiny Reflow Controller is entirely at
 * your own risk, for which we shall not be liable.
 * 
 * 
 * Copyright (C) 2020, Charles Fourneau
 *
 * based on original Tiny Reflow Controller code available at
 * https://github.com/rocketscream/TinyReflowController
 * 
 * 		(C) 2019, Rocket Scream Electronics, Lim Phang Moh
 *          Website: www.rocketscream.com
 * 
 * This Tiny Reflow Controller firmware is released under the
 * Creative Commons Share Alike v4.0 license (CC-BY-SA)
 * 
 * See LICENSE file or http://creativecommons.org/licenses/by-sa/4.0/
 * 
 * You are free to take this piece of code, use it and modify it.
 * All we ask is attribution including the supporting libraries used in this
 * firmware.
 * 
 * Used libraries include
 *  - PID by Brett Beauregard
 *  - MAX31856 by Limor Fried of Adafruit
 *  - U8g2 by Olikraus
 *  - TimerOne by Paul Stoffregen
 * 
 * See README for detailed credits
 *
 ******************************************************************************/

#include <Arduino.h>

// Global libs
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_MAX31856.h> 
#include <PID_v1.h>
#include <U8g2lib.h>
#include <TimerOne.h>
// Local/project libs
#include <Button.h>
#include <Buzzer.h>
// Project headers
#include "config.h"
#include "reflow.h"

// PID & other logic control variables
double setpoint;
double input;
double output;
uint32_t windowStartTime;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Reflow profile type
uint8_t selectedProfile;
// timers
uint32_t lastSensorReading;
uint32_t lastLcdUpdate;
uint32_t timerSoak;
// Seconds timer
// while typical reflow is <10min, larger values are needed for baking (several hours/days)
uint32_t timerSeconds;
uint32_t timerUpdate;
// Thermocouple fault status
uint8_t fault;
// Reflow curve plot data
uint8_t plotTemperature[PLOT_MAX_POINTS];
uint8_t plotX;
uint8_t plotNbPoints;
// Display data box overlay
displayOverlayState_t showDataOverlay;

// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT, P_ON_E, DIRECT);

// LCD interface
// U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0); // 128 bytes framebuffer
U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0); // 256 bytes framebuffer

// MAX31856 thermocouple interface
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(PIN_MAX31856_CS);

// Buttons
Button switch_1(PIN_SW_START_STOP);
Button switch_2(PIN_SW_LF_PB);

// Buzzer
ActiveBuzzer buzzer(PIN_BUZZER);

// ISR for debouncing and buzzer timer
void timerIsr()
{
	switch_1.update();
	switch_2.update();
	buzzer.update();
}

// helper functions
// not sure inline works on arduino though...
inline void heater_on() { digitalWrite(PIN_SSR, HIGH); }
inline void heater_off() { digitalWrite(PIN_SSR, LOW); }
inline void fan_on() { digitalWrite(PIN_FAN, HIGH); }
inline void fan_off() { digitalWrite(PIN_FAN, LOW); }
inline void led_on() { digitalWrite(PIN_LED, HIGH); }
inline void led_off() { digitalWrite(PIN_LED, LOW); }
inline void led_toggle() { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); }

/**
 * SETUP
 */
void setup()
{
	// Serial communication at 115200 bps
	Serial.begin(115200);
	Serial.println(F("*\n* Tiny Reflow Controller\n*"));
  
	// buttons
	switch_1.begin();
	switch_2.begin();

	// Buzzer initialization
	buzzer.begin();

	// Input processing and debouncing
	Timer1.initialize(5000); // 5ms
	Timer1.attachInterrupt(timerIsr);

	// SSR pin initialization to ensure reflow oven is off
  	pinMode(PIN_SSR, OUTPUT);
  	heater_off();

	// Fan pin initialization
  	pinMode(PIN_FAN, OUTPUT);
	fan_off();

  	// LED pins initialization and turn on upon start-up (active high)
  	pinMode(PIN_LED, OUTPUT);
  	led_on();

  	// Initialize thermocouple interface
  	thermocouple.begin();
  	thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);
	thermocouple.setConversionMode(MAX31856_ONESHOT);

	// Check current selected reflow profile
	selectedProfile = 0;
#if ENABLE_EEPROM == 1
	selectedProfile = EEPROM.read(EEPROM_PROFILE_TYPE_ADDRESS);
#endif
	if (selectedProfile >= COUNT_REFLOW_PROFILES) {
		// Default to lead-free profile
#if ENABLE_EEPROM == 1
		EEPROM.write(EEPROM_PROFILE_TYPE_ADDRESS, 0);
#endif
		selectedProfile = 0;
	}

	// Tell the PID to range between 0 and the full window size
	reflowOvenPID.SetOutputLimits(0, PID_WINDOW_SIZE);
	reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
	reflowOvenPID.SetMode(MANUAL);

	// Start-up splash
#if ENABLE_BUZZER == 1
	buzzer.on();
#endif
	u8g2.begin();
	u8g2.setFont(U8G2_FONT_LARGE);
	u8g2.setFontPosTop();
	u8g2.firstPage();
	do {
		u8g2.setCursor(0,0);
		u8g2.print(F("Tiny"));
		u8g2.setCursor(0,16);
		u8g2.print(F("Reflow"));
		u8g2.setCursor(0,32);
		u8g2.print(F("Controller"));
	} while ( u8g2.nextPage() );
	delay(2000);

	// data overlay is off by default
	showDataOverlay = OVERLAY_OFF;

	// Turn off LED (active high)
	led_off();

	// Initialize time keeping variable
	lastSensorReading = 0; // force initial reading
	// Initialize LCD update timer
	lastLcdUpdate = 0;
}

/**
 * LOOP
 */
void loop()
{
	// Current time
	uint32_t now = millis();

	// update switches - if not using interrupts
	// but Adafruit_MAX31856 has a delay that f*cks this up
	// version >1.2.0 has a manual mode than can solve this...
	//switch_1.update();
	//switch_2.update();

	// Time to read thermocouple?
	if ((now - lastSensorReading) > SENSOR_SAMPLING_TIME) {
		// Read thermocouple next sampling period
		lastSensorReading = now;

		// Read current temperature
		// Is blocking (delay) for ~100ms (or 250ms for Adafruit_MAX31856 < 1.2.0)
		// we could also use newer non-blocking "manual" read method... meh
		input = thermocouple.readThermocoupleTemperature();
		// Check for thermocouple fault
		fault = thermocouple.readFault();

		// If any thermocouple fault is detected
		if ((fault & MAX31856_FAULT_CJRANGE) ||
			(fault & MAX31856_FAULT_TCRANGE) ||
			(fault & MAX31856_FAULT_CJHIGH) ||
			(fault & MAX31856_FAULT_CJLOW) ||
			(fault & MAX31856_FAULT_TCHIGH) ||
			(fault & MAX31856_FAULT_TCLOW) ||
			(fault & MAX31856_FAULT_OVUV) ||
			(fault & MAX31856_FAULT_OPEN))
		{
			// Illegal operation
			reflowState = REFLOW_STATE_ERROR;
			reflowStatus = REFLOW_STATUS_OFF;
			Serial.print(F("TC Error: ")); Serial.println(fault, HEX);
		}

		// If reflow process is on going
		if (reflowStatus == REFLOW_STATUS_ON) {
			// Toggle red LED as system heart beat
			led_toggle();
			// store plot data every 3s
			if (timerSeconds % 3 == 0) {
				if (plotNbPoints < PLOT_MAX_POINTS) {
					// count number of points
					plotNbPoints++;
				}
				plotTemperature[plotX] = (uint8_t)map(input, 0, 250, 63, 19);
				// if we reached the end of the table start at beginning
				plotX = (plotX == (PLOT_MAX_POINTS - 1)) ? 0 : plotX + 1;
			}
			// Increase seconds timer for reflow curve plot
			timerSeconds++;
			// Send temperature and time stamp to serial
			Serial.print(timerSeconds);
			Serial.print(F(","));
			Serial.print(setpoint);
			Serial.print(F(","));
			Serial.print(input);
			Serial.print(F(","));
			Serial.println(output);
		}
		else {
			// Turn off red LED
			led_off();
		}
	}

	now = millis();
	if ((now - lastLcdUpdate) > LCD_UPDATE_RATE) {
		// Update LCD in the next 100 ms
		lastLcdUpdate = now;

		u8g2.firstPage();
		do {
			u8g2.setFont(U8G2_FONT_LARGE);
			u8g2.setCursor(1,0);
			u8g2.print(reflowStateString[reflowState]);

			u8g2.setFont(U8G2_FONT_SMALL);
			u8g2.setCursor((SCREEN_WIDTH - 3) - (2 * U8G2_FONT_SMALL_WIDTH), 0);
			u8g2.print(reflowProfiles[selectedProfile].id);

			// Temperature markers
			u8g2.setCursor(0, 18);
			u8g2.print(F("250"));
			u8g2.setCursor(0, 36);
			u8g2.print(F("150"));
			u8g2.setCursor(0, 54);
			u8g2.print(F("50"));
			// Draw temperature and time axis
			u8g2.drawVLine(X_AXIS_START - 1, 16, SCREEN_HEIGHT - 16);
			u8g2.drawHLine(X_AXIS_START, 63, SCREEN_WIDTH - X_AXIS_START - 2);

			if (reflowState == REFLOW_STATE_ERROR) {
				// If currently in error state
				u8g2.setCursor(85, 8);
				u8g2.print(F("TC Error"));
			}
			else {
				// Right align temperature reading
				// adjust with width of selected font
				if (input < 10) // X.XX°C
					u8g2.setCursor(95, 8);
				else if (input < 100) // XX.XX°C
					u8g2.setCursor(90, 8);
				else // XXX.XX°C
					u8g2.setCursor(85, 8);
				// Display current temperature
				u8g2.print(input);
				u8g2.print("\xB0"); u8g2.print("C");
			}
		
			// draw reflow curve
			uint8_t plotXOffset;
			if (plotNbPoints < PLOT_MAX_POINTS) {
				plotXOffset = 0;
			}
			else {
				plotXOffset = plotX;
			}
			for (uint8_t x = 0; x < plotNbPoints; x++) {
				u8g2.drawPixel(x + X_AXIS_START, plotTemperature[(x + plotXOffset) % PLOT_MAX_POINTS]);
			}

			if (showDataOverlay != OVERLAY_OFF) {
				u8g2.setDrawColor(0);
				u8g2.drawBox(37,16,64,37); // clear the background
				u8g2.setDrawColor(1);
				u8g2.drawFrame(37,16,64,37);
				u8g2.setFont(U8G2_FONT_SMALL);
				if (showDataOverlay == OVERLAY_VARS) {
					u8g2.setCursor(40, 18);
					u8g2.print(F("t: ")); u8g2.print(timerSeconds); u8g2.print(F("s"));
					u8g2.setCursor(40, 26);
					u8g2.print(F("S: ")); u8g2.print(setpoint); u8g2.print("\xB0"); u8g2.print("C");
					u8g2.setCursor(40, 34);
					u8g2.print(F("T: ")); u8g2.print(input); u8g2.print("\xB0"); u8g2.print("C");
					u8g2.setCursor(40, 42);
					u8g2.print(F("O: ")); u8g2.print(output);
				}
				else if (showDataOverlay == OVERLAY_TUNINGS) {
					u8g2.setCursor(40, 18);
					u8g2.print(F("kp: ")); u8g2.print(reflowOvenPID.GetKp());
					u8g2.setCursor(40, 26);
					u8g2.print(F("ki: ")); u8g2.print(reflowOvenPID.GetKi());
					u8g2.setCursor(40, 34);
					u8g2.print(F("kd: ")); u8g2.print(reflowOvenPID.GetKd());
				}
				
			}

		// Update screen
		} while ( u8g2.nextPage() );
	}

	// Reflow oven controller state machine
	now = millis();
	switch (reflowState) {
	case REFLOW_STATE_IDLE:
		// always display curve in IDLE mode
		showDataOverlay = OVERLAY_OFF;

		// If oven temperature is still above room temperature
		if (input >= TEMPERATURE_ROOM) {
			reflowState = REFLOW_STATE_TOO_HOT;
		}
		else {
			// If switch is pressed to start reflow process
			if (switch_1.getState() == Button::Clicked) {
				// beep
#if ENABLE_BUZZER == 1
				buzzer.on(500u);
#endif
				// store used profile
#if ENABLE_EEPROM == 1
				EEPROM.write(EEPROM_PROFILE_TYPE_ADDRESS, selectedProfile);
#endif
				Serial.println(F("Reflow started !"));
				// Send header for CSV file
				Serial.println(F("Time,Setpoint,Input,Output"));
				// Intialize seconds timer for serial debug information
				timerSeconds = 0;
				// Initialize reflow plot update timer
				for (uint8_t x = 0; x < PLOT_MAX_POINTS; x++) {
					plotTemperature[x] = 0;
				}
				// Initialize index for average temperature array used for reflow plot
				plotX = 0;
				plotNbPoints = 0;
				// Initialize PID control window starting time
				windowStartTime = now;
				// Ramp up to minimum soaking temperature
				setpoint = reflowProfiles[selectedProfile].soakStartTemperature;
				// Turn the PID on
				reflowOvenPID.SetMode(AUTOMATIC);
				if (reflowProfiles[selectedProfile].soakEndTemperature == 0) {
					// If no SOAK_END_TEMPERATURE (0) -> Bake
					reflowOvenPID.SetTunings(PID_KP_BAKE, PID_KI_BAKE, PID_KD_BAKE, P_ON_E);
					reflowState = REFLOW_STATE_BAKE;
				}
				else {
					// ... or preheat for a full reflow profile
					reflowOvenPID.SetTunings(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT, P_ON_E);
					reflowState = REFLOW_STATE_PREHEAT;
				}
			}
		}
		break;

	case REFLOW_STATE_PREHEAT:
		reflowStatus = REFLOW_STATUS_ON;
		// If minimum soak temperature is achieved
		if (input >= reflowProfiles[selectedProfile].soakStartTemperature) {
			// Set PID parameters for soaking ramp
			reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
			// Chop soaking period into smaller sub-period
			timerSoak = now + reflowProfiles[selectedProfile].soakMicroPeriod * 1000u;
			// Ramp up to first section of soaking temperature
			setpoint = reflowProfiles[selectedProfile].soakStartTemperature + SOAK_TEMPERATURE_STEP;
			// Proceed to soaking state
			reflowState = REFLOW_STATE_SOAK;
			buzzer.on(300u);
		}
		break;

	case REFLOW_STATE_SOAK:
		// If micro soak temperature is achieved
		if ((now - timerSoak) >= reflowProfiles[selectedProfile].soakMicroPeriod * 1000u) {
			timerSoak = now;
			// Increment micro setpoint
			setpoint += SOAK_TEMPERATURE_STEP;
			if (setpoint > reflowProfiles[selectedProfile].soakEndTemperature) {
				// Set agressive PID parameters for reflow ramp
				reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
				// Ramp up to first section of soaking temperature
				setpoint = reflowProfiles[selectedProfile].reflowMaxTemperature;
				// Proceed to reflowing state
				reflowState = REFLOW_STATE_REFLOW;
				buzzer.on(300u);
			}
		}
		break;

	case REFLOW_STATE_REFLOW:
		// We need to avoid hovering at peak temperature for too long
		// Crude method that works like a charm and safe for the components
		if (input >= (reflowProfiles[selectedProfile].reflowMaxTemperature - 5)) {
			// Set PID parameters for cooling ramp
			reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
			// Ramp down to minimum cooling temperature
			setpoint = reflowProfiles[selectedProfile].coolMinTemperature;
			// Proceed to cooling state
			reflowState = REFLOW_STATE_COOL;
			buzzer.on(500u);
		}
		break;

	case REFLOW_STATE_COOL:
#if ENABLE_FAN == 1
		// TODO: check we do not exceed cooling rate (6°C/min ?)
		fan_on();
#endif
		// If minimum cool temperature is achieved
		if (input <= reflowProfiles[selectedProfile].coolMinTemperature) {
			// Turn off reflow process
			reflowStatus = REFLOW_STATUS_OFF;
			// turn PID off
			reflowOvenPID.SetMode(MANUAL);
			// Proceed to reflow Completion state
			reflowState = REFLOW_STATE_COMPLETE;
		}
		break;

	case REFLOW_STATE_COMPLETE:
		Serial.println(F("Finished."));
#if ENABLE_BUZZER == 1
		// Turn on buzzer to indicate completion
		buzzer.on(1000u);
#endif
		// Reflow process ended
		reflowState = REFLOW_STATE_IDLE;
		break;

	case REFLOW_STATE_TOO_HOT:
		// If oven temperature drops below room temperature
		if (input < TEMPERATURE_ROOM) {
			// Ready to reflow
			reflowState = REFLOW_STATE_IDLE;
		}
		break;

	case REFLOW_STATE_BAKE:
		reflowStatus = REFLOW_STATUS_ON;
		// Nothing to do... stay here until user abort 
		break;

	case REFLOW_STATE_ERROR:
		// Check for thermocouple fault
		fault = thermocouple.readFault();

		// If thermocouple problem is still present
		if ((fault & MAX31856_FAULT_CJRANGE) ||
			(fault & MAX31856_FAULT_TCRANGE) ||
			(fault & MAX31856_FAULT_CJHIGH) ||
			(fault & MAX31856_FAULT_CJLOW) ||
			(fault & MAX31856_FAULT_TCHIGH) ||
			(fault & MAX31856_FAULT_TCLOW) ||
			(fault & MAX31856_FAULT_OVUV) ||
			(fault & MAX31856_FAULT_OPEN))
		{
			// Wait until thermocouple wire is connected
			reflowState = REFLOW_STATE_ERROR;
		}
		else {
			// Clear to perform reflow process
			reflowState = REFLOW_STATE_IDLE;
		}
		break;
	
	default:
		// We should not be here
		;
	}

	// Switch 1 - Hold >1s to stop reflow
	if (switch_1.getState() == Button::Held)	{
		// If currently reflow process is on going
		if (reflowStatus == REFLOW_STATUS_ON) {
			// Button press is for cancelling
			// turn PID off
			reflowOvenPID.SetMode(MANUAL);
			// Turn off reflow process
			reflowStatus = REFLOW_STATUS_OFF;
			// Reinitialize state machine
			reflowState = REFLOW_STATE_IDLE;
			// beep !
#if ENABLE_BUZZER == 1
			buzzer.on(1000u);
#endif
			// Serial
			Serial.println(F("Stopped by user"));
		}
	}

	// Switch 2 is pressed
	if (switch_2.getState() == Button::Clicked) {
		// Only can switch reflow profile during idle
		if (reflowState == REFLOW_STATE_IDLE) {
			selectedProfile = (selectedProfile + 1) % COUNT_REFLOW_PROFILES;
		}
		else {
			showDataOverlay = (displayOverlayState_t)((showDataOverlay + 1) % COUNT_OVERLAY_STATES);
		}
	}
	
	// PID computation and SSR control
	if (reflowStatus == REFLOW_STATUS_ON) {
		now = millis();
		reflowOvenPID.Compute();
		if ((now - windowStartTime) > PID_WINDOW_SIZE) {
			// Time to shift the Relay Window
			windowStartTime += PID_WINDOW_SIZE;
		}
		if (output > (now - windowStartTime))
			heater_on();
		else
			heater_off();
	}
	else {
		// Reflow oven process is off, ensure oven is off
		heater_off();
		fan_off();
	}
}


