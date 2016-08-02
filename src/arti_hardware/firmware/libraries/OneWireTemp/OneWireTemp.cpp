#include <OneWireTemp.h>

OneWireTemp::OneWireTemp(){

}
OneWireTemp::~OneWireTemp(){
    delete sensor_;
}

OneWireTemp::OneWireTemp(int pin) {
    pin_  = pin;
    sensor_ = new OneWire (pin_);
}

void OneWireTemp::initialize(int pin){
    pin_ = pin;
    sensor_ = new OneWire(pin_);
}

float OneWireTemp::getTemp(){

    if (sensor_ == NULL){
        return -1000;
    }

    byte data[12];
    byte addr[8];

    if ( !sensor_->search(addr)) {
        //no more sensors on chain, reset search
        sensor_->reset_search();
        return -1000;
    }

    if ( OneWire::crc8( addr, 7) != addr[7]) {
        // Serial.println("CRC is not valid!");
        return -1000;
    }

    if ( addr[0] != 0x10 && addr[0] != 0x28) {
        // Serial.print("Device is not recognized");
        return -1000;
    }

    sensor_->reset();
    sensor_->select(addr);
    sensor_->write(0x44, 1); // start conversion, with parasite power on at the end

    byte present = sensor_->reset();
    sensor_->select(addr);
    sensor_->write(0xBE); // Read Scratchpad


    for (int i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = sensor_->read();
    }

    sensor_->reset_search();

    byte MSB = data[1];
    byte LSB = data[0];

    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    float TemperatureSum = tempRead / 16;

    return TemperatureSum;
}