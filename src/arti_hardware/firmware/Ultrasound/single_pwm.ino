/*
Test code for the Arduino Uno
Written by Tom Bonar for testing
Sensors being used for this code are the MB70X0 from MaxBotix
*/
const int pwPin1 = 2;

long sensor1, cm;

void setup () {
  Serial.begin(9600);
  pinMode(pwPin1, INPUT);
}

void read_sensor(){
  sensor1 = pulseIn(pwPin1, HIGH);
  cm = sensor1/58;
}

void loop () {
  read_sensor();
  printall();
  delay(100);
}

void printall(){
  Serial.print("S1");
  Serial.print(" = ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
}