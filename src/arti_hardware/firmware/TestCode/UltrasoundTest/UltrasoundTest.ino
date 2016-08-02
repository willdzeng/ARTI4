#include <Arduino.h>
#include <Ultrasound.h>


Ultrasound us;
void setup() {
	Serial.begin(9600);
	byte ultra_value_pins[] = {0, 1};
	byte ultra_trigger_pin = 52;
	us.initialize(ultra_value_pins, sizeof(ultra_value_pins), ultra_trigger_pin);
}

void loop() {
	us.readValue();
	us.printValue();
	delay(300);
}