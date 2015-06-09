#include "Arduino.h"
#include "Wire.h"
#include "RFduinoBLE.h"

float read_temp() {
  int error = 0;

  // Read the two bytes of data in the register
  Wire.beginTransmission(0x40); 
  Wire.write(0x03);
  Wire.requestFrom(0x40, 2);

  byte data[2];
  data[1] = Wire.read();
  data[0] = Wire.read();
  error = Wire.endTransmission();

  if(error != 0) {
    Serial.print("No device found at 0x40. ");
    Serial.print("Error Code: ");
    Serial.println(error, DEC);
    //return;
  }

  if(data[0] & 0x1) {
    Serial.println("Invalid data");
    return 0xFF;
  }

  int regVal = (data[1] << 8) | data[0];
  return (regVal >> 2) * 0.03125;
}

int main() {
  init();

  setup();

  for(;;) {
    loop();
  }

  return 0;
}

void setup() {
  //RFduino_systemReset();
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  delay(1000);
 
  float temp = read_temp();

  Serial.print("Temperature: ");
  Serial.print(temp, DEC);
  Serial.print("C");
  Serial.print("\n\r");
}

