#include <OneWireTemp.h>

int DS18S20_Pin = 22; //DS18S20 Signal pin on digital 2
OneWireTemp ds(DS18S20_Pin);

void setup(void) {
    Serial.begin(9600);
}

void loop(void) {
    float temperature = ds.getTemp();
    Serial.println(temperature);

    delay(100); //just here to slow down the output so it is easier to read

}