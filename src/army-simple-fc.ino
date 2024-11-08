#include <Arduino.h>
#include <Servo.h>

constexpr int moto1_servo_pin = PA6;
constexpr int moto2_servo_pin = PA7;
constexpr int moto3_servo_pin = PB0;
constexpr int moto4_servo_pin = PB1;

// constexpr int test_port1 = PA6;

// Motor servos
Servo motor1_servo;
Servo motor2_servo;
Servo motor3_servo;
Servo motor4_servo;

void setup() {
    // put your setup code here, to run once:
    Serial2.begin(115200);
    Serial2.printf("Booting...\n");

    // Init RF UART
    Serial3.begin(420000);

    // Init motors servos and set to 0
    motor1_servo.attach(moto1_servo_pin);
    motor1_servo.write(0);
    motor2_servo.attach(moto2_servo_pin);
    motor2_servo.write(0);
    motor3_servo.attach(moto3_servo_pin);
    motor3_servo.write(0);
    motor4_servo.attach(moto4_servo_pin);
    motor4_servo.write(0);

}

unsigned long currentMillis = 0;

void loop() {
    static int rc_counter = 0;

    while(Serial3.available()) {
        char c = Serial3.read();
        rc_counter++;
        // Serial2.write(c);
    }

    if(millis() - currentMillis > 1000) {
        currentMillis = millis();
        Serial2.printf("RC: %d\n", rc_counter);
        rc_counter = 0;
    }

}
