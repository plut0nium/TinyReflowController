# Button Debouncing Library
> Simple single pushbutton debouncing

## Usage

```C++
#include <Button.h>

Button btn1(PIN_BTN1);

(...)

void setup()
{
	btn1.begin();

    (...)
}
```

`btn1.update()` must be then called at regular interval, either in your main `loop()` or in an interrupt-based routine.

In your main loop, you can read the button state:
```C++
ButtonState_t bs = btn1.getState()
```

State can be:
```C++
Button::Open
Button::Clicked
Button::DoubleClicked
Button::Held
Button::Released // after a Held state
```

Please note that reading the state of the button will reset it to `Open`, with the exception of `Held` that will remain until button is released.

## License

- Copyright (C) 2020, Charles Fourneau

based on ClickEncoder code available at https://github.com/0xPIT/encoder

- Copyright (c) 2010-2014 karl@pitrich.com

```
BSD license
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
