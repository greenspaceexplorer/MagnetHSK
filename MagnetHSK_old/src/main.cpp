#include <main.h>
#include <SPI.h>

void setup(void) {
  Serial.begin(9600);

}

void loop(void) {
    Serial.println("Testing...");
    delay(1000);
}
