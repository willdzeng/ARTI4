/*Feel free to use this code.
Author: Tom Bonar
Date: 12-11-2013
Analog pin 1 for reading in the analog voltage from the MaxSonar device.
This variable is a constant because the pin will not change throughout execution of this code.*/
const int anPin1 = 0;
const int anPin2 = 1;
const int anPin3 = 2;
const int anPin4 = 3;
const int triggerPin = 52;
long anVolt1, anVolt2, anVolt3, anVolt4, sensor1, sensor2, sensor3, sensor4;

void setup() {
  //This opens up a serial connection to shoot the results back to the PC console
  Serial.begin(9600);
  pinMode(triggerPin,OUTPUT);
  delay(200); //Gives time for the sensors to boot and calibrate
}

void start_sensor(){
  digitalWrite(triggerPin,HIGH);
  delay(1);
  digitalWrite(triggerPin,LOW);
}

void read_sensor(){
  //Used to read in the analog voltage output that is being sent by the XL-MaxSonar device.
  //Scale factor is (Vcc/1024) per  2 centimeter. A 5V supply yields ~4.9mV/2 cm for long range sensors
  anVolt1 = analogRead(anPin1);
  anVolt2 = analogRead(anPin2);
//  anVolt3 = analogRead(anPin3);/
  // anVolt4 = analogRead(anPin4);
  sensor1 = anVolt1*2;
  sensor2 = anVolt2*2;
//  sensor3 = anVolt3*2;/
  // sensor4 = anVolt3*2;
}

void printall() {
  Serial.print("S1");
  Serial.print("=");
  Serial.print(sensor1);
  Serial.print("cm");

  Serial.print(" ");
  Serial.print("S2");
  Serial.print("=");
  Serial.print(sensor2);
  Serial.print("cm");

  Serial.print(" ");
  Serial.print("S3");
  Serial.print("=");
  Serial.print(sensor3);
  Serial.print("cm");

  // Serial.print(" ");
  // Serial.print("S3");
  // Serial.print("=");
  // Serial.print(sensor4);
  // Serial.print("cm");

  Serial.println();
}

void loop () {
  start_sensor();
  read_sensor();
  printall();
  delay(300); // This delay time changes by 100 for every sensor in the chain.  For 4 sensors this will be 400
}
