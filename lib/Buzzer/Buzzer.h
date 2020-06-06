/**
 * 
 * ActiveBuzzer
 * A (very) simple library for active buzzer management
 * 
 * MIT License
 * 
 * Copyright © 2020 Charles Fourneau
 *
 */

#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <Arduino.h>

class ActiveBuzzer
{
public:
	ActiveBuzzer(uint8_t buzzerPin);

public:
	void begin();
	void on(uint16_t time = 250u);
	void off();
	void update();

private:
	uint8_t pin;
	uint32_t startTime;
	uint16_t timer;
	bool isActive;
};

#endif // _BUZZER_H_

