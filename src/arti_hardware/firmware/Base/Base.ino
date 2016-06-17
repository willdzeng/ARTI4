/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <Arduino.h>
#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(52, 50);
Encoder knobRight(53, 51);
//   avoid using pins with LEDs attached

#include <Sabertooth.h>
Sabertooth ST(128);

int left = 0; // store motor value
int right = 0; // store voltage value
long baud_rate = 9600;
int time_out = 300;
String inString = "";

long positionLeft  = -999;
long positionRight = -999;

bool parseString(String str, int& left, int& right);

void setup() {
    Serial.begin(baud_rate);
    Serial.println("TwoKnobs Encoder Test:");
    // SabertoothTXPinSerial.begin(baud_rate);
    ST.setBaudRate(baud_rate);
    ST.setTimeout(time_out);
}

void loop() {
    long newLeft, newRight;
    newLeft = knobLeft.read();
    newRight = knobRight.read();
    if (newLeft != positionLeft || newRight != positionRight) {
        Serial.println();
        Serial.print("Left = ");
        Serial.print(newLeft);
        Serial.print(", Right = ");
        Serial.print(newRight);
        Serial.println();
        positionLeft = newLeft;
        positionRight = newRight;
    }
    // check if serial available
    if (Serial.available() > 0) {
        inString = Serial.readStringUntil('\n');
        if (parseString(inString, left, right)) {
            ST.motor(1, left);
            ST.motor(2, right);
        }
        inString = "";
        left = 0;
        right = 0;
    }
}

bool parseString(String str, int& left, int& right) {
    int id = -1;
    for (int i = 0; i < str.length(); i++) {
        if (str[i] == ' ') {
            id = i;
            break;
        }
    }
    if (id == -1) {
        // Serial.print("\n-1, Wrong Format Input, Use 'xxx yyy'\n");
        return 0;
    }
    String leftstr = str.substring(0, id);
    String rightstr = str.substring(id + 1, str.length());
    left = leftstr.toInt();
    right = rightstr.toInt();
    return 1;
}