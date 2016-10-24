/* This is the hardware Arduino code for ARTI from Transcend Robotics
 * transcend.ai
 *
 * 
 */
#include <Arduino.h>

// #define ENCODER_OPTIMIZE_INTERRUPTS
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Rotary.h>
// whether to use odometry
bool use_odom = false;
// whether to use ultrasound
bool use_ultrasound = false;
// whether to use temperature
bool use_temp = false;

Rotary knobLeft, knobRight;

#include <Sabertooth.h>
Sabertooth ST(128, Serial1);

#include <Ultrasound.h>
Ultrasound US;

byte ultra_value_pins[] = {0, 1};
byte ultra_trigger_pin = 31;
float ultrasound_frequency = 5.0;

#include <MultiTemp.h>
byte temp_value_pins[] = {46, 47, 48, 49, 50, 51};
float temp_frequency = 5.0;
MultiTemp MT;

int left = 0; // store motor value
int right = 0; // store voltage value
long baud_rate = 9600;
int time_out = 500;
String use_str = "";
String tmp_str = "";
String data_str = "";

long positionLeft  = 0;
long positionRight = 0;

bool parseMotorCmd(String str, int& left, int& right);
void processOdom();
void processCmd();

void setup() {
	Serial.begin(baud_rate);
	Serial1.begin(baud_rate);
	ST.setBaudRate(baud_rate);
	ST.setTimeout(time_out);

	if (use_odom) {
		knobLeft.initialize(54, 55);
		knobRight.initialize(56, 57);
	}

	if (use_ultrasound) {
		// initialzed Ultrasound
		US.initialize(ultra_value_pins, sizeof(ultra_value_pins), ultra_trigger_pin, ultrasound_frequency);
	}

	if (use_temp) {
		// initialize Temprature sensor
		MT.initialize(temp_value_pins, sizeof(temp_value_pins), temp_frequency);
	}

}

void loop() {
	// if use odometry
	if (use_odom) {
		processOdom();
	}
	// if use ultrasound
	if (use_ultrasound) {
		if (US.isReady()) {
			US.readValue();
			US.printValue();
		}
	}
	// if use tempraure sensor
	if (use_temp) {
		if (MT.isReady()) {
			MT.readValue();
			MT.printValue();
		}
	}
	// each loop must read the motor cmd
	processCmd();
}

void processCmd() {
	// read motor input
	if (Serial.available() > 0) {
		tmp_str = Serial.readStringUntil('\r');
		use_str = Serial.readStringUntil('\n');
		// check token type
		if (use_str[0] == '$') {
			if (use_str.substring(1, 6) == "MOTO,") {
				data_str = use_str.substring(6);
			}
		}
		// parse the cmd
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

void processOdom() {
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
	return 1;
}
