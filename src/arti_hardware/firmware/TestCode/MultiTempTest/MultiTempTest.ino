#include <MultiTemp.h>

byte temp_value_pins[] = {22, 24};
MultiTemp temps;

void setup(void) {
    Serial.begin(9600);
    temps.initialize(temp_value_pins, 2, 5.0);
}

void loop(void) {
    temps.readValue();
    temps.printValue();
    delay(100); //just here to slow down the output so it is easier to read
}