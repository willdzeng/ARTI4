// Send
#include <RS485.h>
#include <SoftwareSerial.h>


char Message[maxMsgLen+1] ;


void setup()
{
  Serial.begin(2400);
  Serial.println("System Startup - Sender");

  RS485_Begin(2400);
}

void loop()
{
  // strcpy(Message,"A0 00 00 04 00 1F AF 14");
  strcpy(Message,"A0 01 00 04 00 1F AF 15");

  if(RS485_SendMessage(Message,fWrite,ENABLE_PIN))
  {
    Serial.print("Sending:");
    Serial.println(Message);
  }  

  delay(1000);  
}
