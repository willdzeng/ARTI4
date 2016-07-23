#include "Ultrasound.h"
// in oder to avoid pointer of arry problem, we use following global varriables as convention:
// thoes variable must be defined before class
// read value pins from: ultra_value_pins_
// read trigger pin from: trigger_pin_
// store the value in ultra_values_

Ultrasound::Ultrasound() {}

void Ultrasound::initialize() {
    num_ = sizeof(ultra_value_pins_);
    // initialize value and pins
    int i = 0;
    while (i < num_) {
        ultra_values_[i] = 0;
        i++;
    }
    pinMode(trigger_pin_, OUTPUT);
    delay(200);
}

void Ultrasound::triggerSensor() {
    digitalWrite(trigger_pin_, HIGH);
    delay(1);
    digitalWrite(trigger_pin_, LOW);
}

void Ultrasound::readValue() {
    digitalWrite(trigger_pin_, HIGH);
    delay(1);
    digitalWrite(trigger_pin_, LOW);
    Serial.println(num_);
    for (int i = 0; i < num_; i++) {
        // Serial.println(ultra_value_pins_[i]);
        ultra_values_[i] = analogRead(ultra_value_pins_[i]) * 2;
    }
}

void Ultrasound::printValue() {
    for (int i = 0; i < num_; i++) {
        Serial.print("S");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(ultra_values_[i]);
        Serial.print("cm  ");
    }
    Serial.println();
}