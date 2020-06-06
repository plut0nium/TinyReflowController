/**
 * This file is part of
 * 
 * ActiveBuzzer
 * A (very) simple library for active buzzer management
 * 
 * MIT License
 * 
 * Copyright © 2020 Charles Fourneau
 *
 */

#include "Buzzer.h"

ActiveBuzzer::ActiveBuzzer(uint8_t buzzerPin):
	pin(buzzerPin),
	isActive(false)
{
	// nothing
}

void ActiveBuzzer::begin() 
{
	pinMode(pin, OUTPUT);
	off(); // make sure it is off
}

void ActiveBuzzer::on(uint16_t time)
{
	isActive = true;
	startTime = millis();
	timer = time;
	digitalWrite(pin, HIGH);
}

void ActiveBuzzer::off()
{
	isActive = false;
	digitalWrite(pin, LOW);
}

void ActiveBuzzer::update()
{
	if (isActive) {
		if ((millis() - startTime) < timer) {
			// timer has not expired yet - do nothing
			return;
		}
	}
	// turn/keep it off
	off();
}




