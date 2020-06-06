/******************************************************************************
 * This file is part of
 * 
 * Button debouncig library
 * 
 * Copyright (C) 2020, Charles Fourneau
 *
 * based on ClickEncoder code available at https://github.com/0xPIT/encoder
 * 
 * 		(c) 2010-2014 karl@pitrich.com
 * 
 * see header file for more information
 * 
 ******************************************************************************/

#include "Button.h"

#define DOUBLE_CLICK_TICKS (DOUBLE_CLICK_TIME / DEBOUNCE_TIME)
#define HOLD_TICKS (HOLD_TIME / DEBOUNCE_TIME)

Button::Button(uint8_t buttonPin, bool enablePullUp, bool activeLow) :
	pin(buttonPin),
	enablePullUp(enablePullUp)
{
	activeState = activeLow ? LOW : HIGH;
}

void Button::begin()
{
	pinMode(pin, enablePullUp ? INPUT_PULLUP : INPUT);
	pushedTicks = 0;
	doubleClickTimer = 0;
	buttonState = Open;
}

void Button::update()
{
	uint32_t now = millis();

	if (now - lastCheck > DEBOUNCE_TIME) {
		lastCheck = now;

		bool pinIsActive = (digitalRead(pin) == activeState);
   
		if (pinIsActive) {	// putton pushed down...
			pushedTicks++;
			if (pushedTicks > HOLD_TICKS) {
				buttonState = Held;
			}
		}
		else { // ...button is released
			if (pushedTicks > 1) { // button has been pushed for more than 1 tick
				if (buttonState == Held) {
					buttonState = Released;
					doubleClickTimer = 0;
				}
				else {
					if (doubleClickTimer > 0) {   // double click timer is running
						buttonState = DoubleClicked;
						doubleClickTimer = 0;
					}
					else {
						doubleClickTimer = DOUBLE_CLICK_TICKS;
					}
				}
			}

			pushedTicks = 0;
		}
	
		if (doubleClickTimer > 0) {
			doubleClickTimer--;
			if (doubleClickTimer == 0) { // timer has expired
				buttonState = Clicked;
			}
		}
	}
}

Button::ButtonState_t Button::getState()
{
	noInterrupts();
	ButtonState_t ret = buttonState;
	if (buttonState != Held) {
		// reset button state if != Held
		buttonState = Open;
	}
	interrupts();
	return ret;
}

