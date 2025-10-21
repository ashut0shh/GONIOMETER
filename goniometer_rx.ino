#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);       // PC serial monitor
  BTSerial.begin(38400);    // HC-05 AT mode baud
  Serial.println("Enter AT commands:");
}

void loop() {
  if (BTSerial.available())
    Serial.write(BTSerial.read());
  if (Serial.available())
    BTSerial.write(Serial.read());
}
