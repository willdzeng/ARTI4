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

Rotary knobLeft(50, 52);
Rotary knobRight(53, 51);

#include <Sabertooth.h>
Sabertooth ST(128, Serial1);

#include <Ultrasound.h>
Ultrasound US;

byte ultra_value_pins[] = {0, 1};
byte ultra_trigger_pin = 49;

int left = 0; // store motor value
int right = 0; // store voltage value
long baud_rate = 9600;
int time_out = 300;
String use_str = "";
String tmp_str = "";
String data_str= "";

long positionLeft  = 0;
long positionRight = 0;

bool parseMotorCmd(String str, int& left, int& right);
void processOdom();

void setup() {
    Serial.begin(baud_rate);
    // Serial.println("TwoKnobs Encoder Test:");
    Serial1.begin(baud_rate);
    // Serial1.begin(baud_rate);
    ST.setBaudRate(baud_rate);
    // ST.autobaud();
    ST.setTimeout(time_out);
    // initialzed Ultrasound
    double ultrasound_frequency = 5;
    US.initialize(ultra_value_pins, sizeof(ultra_value_pins), ultra_trigger_pin, ultrasound_frequency);
}

void loop() {
    // processOdom();

    if (US.isReady()) {
        US.readValue();
        US.printValue();
    }

    // read motor input
    if (Serial.available() > 0) {
        tmp_str = Serial.readStringUntil('\r');
        use_str = Serial.readStringUntil('\n');
        if (use_str[0] == '$'){
            if (use_str.substring(1,6) == "MOTO,"){
                data_str = use_str.substring(6);
            }
        }
        // Serial.print(dataString);
        if (parseMotorCmd(data_str, left, right)) {
            ST.motor(1, left);
            ST.motor(2, right);
        }
        tmp_str = "";
        data_str = "";
        use_str = "";
        left = 0;
        right = 0;
    }
}

void processOdom(){
    // read value
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
        Serial.print("$ODOM,");
        Serial.print(positionLeft);
        Serial.print(",");
        Serial.print(positionRight);
        Serial.print("\n");
        newValue = false;
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
