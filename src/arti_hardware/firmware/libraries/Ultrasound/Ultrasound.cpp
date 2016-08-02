#include "Ultrasound.h"

// in oder to avoid pointer of arry problem, we use following global varriables as convention:
// thoes variable must be defined before class
// read value pins from: ultra_value_pins_
// read trigger pin from: ultra_trigger_pin_
// store the value in ultra_values_

Ultrasound::Ultrasound() {}

Ultrasound::~Ultrasound() {
    delete ultra_value_pins_;
    delete ultra_values_;
}

void Ultrasound::initialize(byte* ultra_value_pins, int size , byte ultra_trigger_pin, double frequency)
{
    ultra_value_pins_ = new byte [size];
    ultra_values_ = new int [size];
    num_ = size;
    for (int i = 0; i < size; i++) {
        ultra_value_pins_[i] = ultra_value_pins[i];
        ultra_values_[i] = 0;
    }
    ultra_trigger_pin_ = ultra_trigger_pin;

    frequency_ = frequency;
    interval_ = 1 / frequency * 1000;

    pinMode(ultra_trigger_pin_, OUTPUT);
    delay(300);
    digitalWrite(ultra_trigger_pin_, HIGH);
    delay(1);
    digitalWrite(ultra_trigger_pin_, LOW);
}

bool Ultrasound::isReady() 
{
    if ( (millis() - time_) > interval_){
        return true;
    }else{
        return false;
    }
}

void Ultrasound::readValue() {
    digitalWrite(ultra_trigger_pin_, HIGH);
    delay(1);
    digitalWrite(ultra_trigger_pin_, LOW);
    for (int i = 0; i < num_; i++) {
        // Serial.print(analogRead(ultra_value_pins_[i]) * 2);
        ultra_values_[i] = analogRead(ultra_value_pins_[i]) * 2;
    }
    time_ = millis();
}

void Ultrasound::printValue() {
    Serial.print("\r$ULTR,");
    for (int i = 0; i < num_; i++) {
        Serial.print(ultra_values_[i]);
        Serial.print(",");
    }
    Serial.print("\n");
}