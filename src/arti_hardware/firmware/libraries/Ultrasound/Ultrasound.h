#ifndef Ultrasound_H
#define Ultrasound_H  
// in oder to avoid pointer of arry problem, we use following global varriables as convention:
// thoes variable must be defined before class
// read value pins from: ultra_value_pins_
// read trigger pin from: trigger_pin_
// store the value in ultra_values_
class Ultrasound {
public:
    Ultrasound() {}

    void initialize();

    void triggerSensor();

    void readValue();

    void printValue();

private:
    int num_;
};
