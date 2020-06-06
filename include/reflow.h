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

#ifndef _REFLOW_H_
#define _REFLOW_H_

// Reflow profiles
#define TEMPERATURE_ROOM 50u
#define SENSOR_SAMPLING_TIME 1000u
#define SOAK_TEMPERATURE_STEP 5 // °C
#define COOL_RATE_MAX 5 // °C/s

struct reflowProfile_t {
	char id[3];
	// all temperatures for a reflow profile should fit in 8-bit integers...
	uint8_t soakStartTemperature;
	uint8_t soakEndTemperature;
	uint8_t reflowStartTemperature;
	uint8_t reflowMaxTemperature;
	uint8_t coolMinTemperature;
	uint8_t soakMicroPeriod; // in seconds
};

/**
 * Reflow profiles
 * In order to define a "baking" profile, just set the first temperature setpoint
 * at the desired value, and all others values at 0.
 */
#define COUNT_REFLOW_PROFILES 4
const reflowProfile_t reflowProfiles[COUNT_REFLOW_PROFILES] = {
	{"LF",150,200,217,250,100,9}, 	// lead-free Sn-Cu-Ag
	{"PB",150,180,185,224,100,10},	// leaded Sn-Pb
	{"LT",100,125,139,175,80,12},	// low-temp Sn-Bi ** NOT TESTED **
	{"BK",80,0,0,0,0,0},			// Bake @ 80°C
};

// PID parameters
#define PID_SAMPLE_TIME 1000u
#define PID_WINDOW_SIZE 2000u
// Preheat stage
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.08
#define PID_KD_PREHEAT 100
// Soaking stage
#define PID_KP_SOAK 200
#define PID_KI_SOAK 0.15
#define PID_KD_SOAK 20
// Reflow
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 10
// Bake
#define PID_KP_BAKE 25
#define PID_KI_BAKE 0.1
#define PID_KD_BAKE 200

// Reflow State Machine
enum reflowState_t
{
	REFLOW_STATE_IDLE,
	REFLOW_STATE_PREHEAT,
	REFLOW_STATE_SOAK,
	REFLOW_STATE_REFLOW,
	REFLOW_STATE_COOL,
	REFLOW_STATE_COMPLETE,
	REFLOW_STATE_TOO_HOT,
	REFLOW_STATE_BAKE,
	REFLOW_STATE_ERROR
};

enum reflowStatus_t
{
	REFLOW_STATUS_OFF,
	REFLOW_STATUS_ON
};

// LCD messages
const char* reflowStateString[] = {
	"Ready",
	"Preheat",
	"Soak",
	"Reflow",
	"Cool",
	"Done!",
	"Hot!",
	"Baking",
	"ERROR"
};

#endif // _REFLOW_H_