/******************************************************************************
 * This file is part of
 * 
 * TINY REFLOW CONTROLLER
 * 
 * Copyright (C) 2020, Charles Fourneau
 *
 * based on original Tiny Reflow Controller code available at
 * https://github.com/rocketscream/TinyReflowController
 * 
 * 		(C) 2019, Rocket Scream Electronics, Lim Phang Moh
 *          Website: www.rocketscream.com
 * 
 * Released under CC-BY-SA 4.0
 * See main.cpp or LICENSE for more information
 * 
 ******************************************************************************/

#ifndef _CONFIG_H_
#define _CONFIG_H_

#define VERSION 2

// Pin assignment
#define PIN_SSR A0
#define PIN_FAN A1
#define PIN_MAX31856_CS 10
#define PIN_LED 4
#define PIN_BUZZER 5
#define PIN_SW_START_STOP 3
#define PIN_SW_LF_PB 2

// Buzzer
#define ENABLE_BUZZER 1

// Fan
#define ENABLE_FAN 0

// EEPROM
#define EEPROM_PROFILE_TYPE_ADDRESS 0
#define ENABLE_EEPROM 0

// Oled display
#define LCD_UPDATE_RATE 500
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define X_AXIS_START 18 // X-axis starting position
#define PLOT_MAX_POINTS (SCREEN_WIDTH - X_AXIS_START -2)

#define U8G2_FONT_LARGE u8g2_font_ncenB10_tr
#define U8G2_FONT_SMALL u8g2_font_profont10_tf
#define U8G2_FONT_SMALL_WIDTH 5 // could use u8g2.getMaxCharWidth() instead

// Display states
#define COUNT_OVERLAY_STATES 3
enum displayOverlayState_t
{
	OVERLAY_OFF,
	OVERLAY_VARS,
	OVERLAY_TUNINGS
};

#endif

