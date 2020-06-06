# ActiveBuzzer
> A (very) simple library for active buzzer management

## Usage

```C++
#include <Buzzer.h>

#define PIN_BUZZER 5 // or whatever pin your buzzer is connected to

ActiveBuzzer buzz(PIN_BUZZER);

void setup() {
    (...)

    buzz.begin();
}

void loop {
    buzz.update(); // or in an ISR if your loop is slow

    (...)

    buzz.on(500); // 500ms, defaults to 250 if omitted
}
```

Don't forget to run `buzz.update()` at regular interval...  
You can also force the buzzer off with `buzz.off()`.

## License

MIT License

Copyright (c) 2020 Charles Fourneau

See LICENSE for more information.
