#include "MultiTemp.h"

// in oder to avoid pointer of arry problem, we use following global varriables as convention:
// thoes variable must be defined before class
// read value pins from: value_pins_
// read trigger pin from: ultra_trigger_pin_
// store the value in values_

MultiTemp::MultiTemp() {}

MultiTemp::~MultiTemp() {
    delete value_pins_;
    delete values_;
    delete sensors_;
}

void MultiTemp::initialize(byte* value_pins, int size, double frequency)
{
    value_pins_ = new byte [size];
    values_ = new float [size];
    sensors_ = new OneWireTemp [size];

    num_ = size;
    for (int i = 0; i < size; i++) {
        value_pins_[i] = value_pins[i];
        values_[i] = 0;
        sensors_[i].initialize(value_pins[i]);
    }

    frequency_ = frequency;
    interval_ = 1 / frequency * 1000;

    delay(200);
}

bool MultiTemp::isReady() 
{
    if ( (millis() - time_) > interval_){
        return true;
    }else{
        return false;
    }
}

void MultiTemp::readValue() {
    for (int i = 0; i < num_; i++) {
        values_[i] = sensors_[i].getTemp();
    }
    time_ = millis();
}

void MultiTemp::printValue() {
    Serial.print("\r$TEMP,");
    for (int i = 0; i < num_; i++) {
        Serial.print(values_[i]);
        Serial.print(",");
    }
    Serial.print("\n");
}