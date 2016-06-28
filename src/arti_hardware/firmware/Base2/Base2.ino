/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <Arduino.h>
// #define ENCODER_OPTIMIZE_INTERRUPTS
#define ENCODER_DO_NOT_USE_INTERRUPTS
//#include <Encoder.h>
#include <Rotary.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
// Encoder knobLeft(50, 52);
// Encoder knobRight(53, 51);
Rotary knobLeft(50, 52);
Rotary knobRight(53, 51);

#include <Sabertooth.h>
// #define USBCON true
Sabertooth ST(128, Serial);

int left = 0; // store motor value
int right = 0; // store voltage value
long baud_rate = 9600;
int time_out = 300;
String dataString = "";
String tmpString = "";

// long positionLeft  = -999;
// long positionRight = -999;

long positionLeft  = 0;
long positionRight = 0;

bool parseMotorCmd(String str, int& left, int& right);

void setup() {
    Serial.begin(baud_rate);
    // Serial.println("TwoKnobs Encoder Test:");
    SabertoothTXPinSerial.begin(baud_rate);
    // Serial1.begin(baud_rate);
    ST.setBaudRate(baud_rate);
    // ST.autobaud();
    ST.setTimeout(time_out);
}

void loop() {
    bool newValue = false;
    unsigned char leftValue, rightValue;
    leftValue = knobLeft.process();
    rightValue = knobRight.process();
    // check left value
    if (leftValue) {
        if (leftValue == DIR_CW) {
            positionLeft++;
        }
        if (leftValue == DIR_CCW) {
            positionLeft--;
        }
        newValue = true;

    }
    // check right value
    if (rightValue) {
        if (rightValue == DIR_CW) {
            positionRight++;
        }
        if (rightValue == DIR_CCW) {
            positionRight--;
        }
        newValue = true;
    }
    // if new value print it.
    if (newValue) {
        Serial.print("\r");
        Serial.print("$ODOMS,");
        Serial.print(positionLeft);
        Serial.print(",");
        Serial.print(positionRight);
        Serial.print(",ODOME");
        Serial.print("\n");
        newValue = false;
    }
    // check if serial available
    if (Serial.available() > 0) {
        tmpString = Serial.readStringUntil('MOTOS,');
        dataString = Serial.readStringUntil('MOTOE\n');
        if (parseMotorCmd(dataString, left, right)) {
            ST.motor(1, left);
            ST.motor(2, right);
        }
        tmpString = "";
        dataString = "";
        left = 0;
        right = 0;
    }
}

bool parseMotorCmd(String str, int& left, int& right) {

    int id1 = -1;
    int id2 = -1;

    id1 = str.indexOf(',');
    id2 = str.indexOf(',', id1 + 1);

    if (id1 == -1 || id2 == -1) {
        Serial.print("\n-1, Wrong Format Input, Use 'xxx yyy'\n");
        return 0;
    }

    String leftstr = str.substring(0, id1);
    String rightstr = str.substring(id1 + 1, id2);
    left = leftstr.toInt();
    right = rightstr.toInt();
    // Serial.println();
    // Serial.println(left);
    // Serial.println(right);
    // left = 0;
    // right = 0;
    return 1;
}
