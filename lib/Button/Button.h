/******************************************************************************
 * 
 * Button debouncig library
 * 
 * Copyright (C) 2020, Charles Fourneau
 *
 * based on ClickEncoder code available at https://github.com/0xPIT/encoder
 * 
 * 		(c) 2010-2014 karl@pitrich.com
 * 
 * BSD license
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include<Arduino.h>

#define DEBOUNCE_TIME 25
#define HOLD_TIME 1000
#define DOUBLE_CLICK_TIME 400

class Button
{
public:
	typedef enum {
		Open = 0,
		Clicked,
		DoubleClicked,
		Held,
		Released,
	} ButtonState_t;

public:
	Button(uint8_t buttonPin, bool enablePullUp=true, bool activeLow=true);

public:
	void begin();
	void update();
	ButtonState_t getState();

private:
	uint8_t pin;
	uint8_t activeState;
	bool enablePullUp;
	ButtonState_t buttonState;
	uint32_t lastCheck;
	uint16_t pushedTicks;
	uint16_t doubleClickTimer;
};

#endif // _BUTTON_H_