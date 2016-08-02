#ifndef Ultrasound_H
#define Ultrasound_H
#include <Arduino.h>
// #include <Time.h>
// in oder to avoid pointer of arry problem, we use following global varriables as convention:
// thoes variable must be defined before class
// read value pins from: ultra_value_pins_
// read trigger pin from: ultra_trigger_pin_
// store the value in ultra_values_
class Ultrasound {
public:
	Ultrasound();
	~Ultrasound();

	void initialize(byte* ultra_value_pins, int size ,byte ultra_trigger_pin, double frequency_);
	void triggerSensor();
	void readValue();
	void printValue();
	bool isReady();

private:
	int num_;
	byte *ultra_value_pins_;
	int *ultra_values_;
	byte ultra_trigger_pin_;
	double frequency_;
	double interval_;
	unsigned long time_;
};

#endif