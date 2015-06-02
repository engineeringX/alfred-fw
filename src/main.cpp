#include "Arduino.h"
#include "RFduinoBLE.h"

int main() {
  init();

  setup();

  for(;;) {
    loop();
  }

  return 0;
}

void setup() {
  RFduinoBLE.advertisementData = "test";
  RFduinoBLE.begin();
}

void loop() {
  RFduino_ULPDelay(INFINITE);
}

