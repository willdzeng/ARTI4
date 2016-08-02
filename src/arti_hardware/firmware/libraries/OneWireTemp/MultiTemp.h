#ifndef MultiTemp_H
#define MultiTemp_H
#include <Arduino.h>
#include <OneWireTemp.h>

// in oder to avoid pointer of arry problem, we use following global varriables as convention:
// thoes variable must be defined before class
// read value pins from: value_pins_
// store the value in values_
class MultiTemp {
public:
	MultiTemp();
	~MultiTemp();

	void initialize(byte* ultra_value_pins, int size, double frequency_);
	void readValue();
	void printValue();
	bool isReady();

private:
	int num_;
	byte *value_pins_;
	float *values_;
	double frequency_;
	double interval_;
	unsigned long time_;
	OneWireTemp* sensors_;
};

#endif