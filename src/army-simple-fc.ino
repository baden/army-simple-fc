#include <Arduino.h>

constexpr int test_port1 = PA6;


void setup() {
    // put your setup code here, to run once:
    Serial2.begin(115200);
    Serial2.printf("Booting...\n");

    pinMode(test_port1, OUTPUT);
    digitalWrite(test_port1, HIGH);
}

void loop() {
    // put your main code here, to run repeatedly:
    Serial2.printf("Hello, world!\n");
    delay(1000);

    digitalWrite(test_port1, HIGH);

    delay(1000);

    digitalWrite(test_port1, LOW);

}
