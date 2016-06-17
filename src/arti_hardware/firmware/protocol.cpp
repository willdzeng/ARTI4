#include <Sabertooth.h>
Sabertooth ST(128);

int left = 0; // store motor value
int right = 0; // store voltage value
int baud_rate = 9600;
int time_out = 1000;
String inString = "";

bool parseString(String str, int& left, int& right);

void setup() {
    SabertoothTXPinSerial.begin(baud_rate);
    ST.setTimeout(time_out);
}

void loop() {
    // check if serial available
    while (Serial.available() > 0) {

        inString = Serial.readStringUntil('\n');

        if (parseString(inString, left, right)) {
            // Serial.print("\n Left: ");
            // Serial.println(left);
            // Serial.print("Right: ");
            // Serial.println(right);
            // Serial.print("\n");
            ST.motor(1, left);
            ST.motor(2, right);
        }

        inString = "";
        left = 0;
        right = 0;
    }
}

bool parseString(String str, int& left, int& right) {
    // parse the string to get the left right command.
    int id = -1;
    for (int i = 0; i < str.length(); i++) {
        if (str[i] == ' ') {
            id = i;
            break;
        }
    }
    // if the formmat is wrong then print error.
    if (id == -1) {
        // Serial.print("\n-1, Wrong Format Input, Use 'xxx yyy'\n");
        return 0;
    }
    // Convert sring to integer.
    String leftstr = str.substring(0, id);
    String rightstr = str.substring(id + 1, str.length());
    left = leftstr.toInt();
    right = rightstr.toInt();

    return 1;

}

