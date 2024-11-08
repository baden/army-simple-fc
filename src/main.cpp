#include <Arduino.h>
#include <Servo.h>

#include <crsf.h>

// Motor servos pins
constexpr int moto1_servo_pin = PA6;
constexpr int moto2_servo_pin = PA7;
constexpr int moto3_servo_pin = PB0;
constexpr int moto4_servo_pin = PB1;

// ADC pins (inpu1-in5/PA5, input2-in10/PB2)
constexpr int adc1_pin = PA5;
constexpr int adc2_pin = PB2;

// Winch direction pins
constexpr int winch_up_pin = PC15;
constexpr int winch_down_pin = PC14;

// Start/stop digital pins
constexpr int starter_pin = PA8;
constexpr int decompressor_pin = PC6;
// Acceleration servo pin
constexpr int acc_servo_pin = PA4;

// Beeper digital pin
constexpr int beeper_pin = PB7;


// TODO: GPS UART & I2C
constexpr int gps_i2c_sda_pin = PA12;
constexpr int gps_i2c_scl_pin = PA11;


// Motor servos
Servo motor1_servo;
Servo motor2_servo;
Servo motor3_servo;
Servo motor4_servo;

Servo acc_servo;

// VTX UART is half-duplex
// https://github.com/AlanAlger001/ExpressLRS/blob/8e3248cc0bfe9b015bf527616cd7259d88ce0f58/src/src/rx_main.cpp#L135
HardwareSerial VtxSerial(USART4, HALF_DUPLEX_ENABLED);
// HardwareSerial CrsfRxSerial(USART1, HALF_DUPLEX_ENABLED);

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

    // Init Winch direction pins
    pinMode(winch_up_pin, OUTPUT);
    digitalWrite(winch_up_pin, LOW);
    pinMode(winch_down_pin, OUTPUT);
    digitalWrite(winch_down_pin, LOW);

    // Init Start/stop pins
    pinMode(starter_pin, OUTPUT);
    digitalWrite(starter_pin, LOW);
    pinMode(decompressor_pin, OUTPUT);
    digitalWrite(decompressor_pin, LOW);
    // Acceleration servo pin
    acc_servo.attach(acc_servo_pin);
    acc_servo.write(0);


    // Beeper pin
    pinMode(beeper_pin, OUTPUT);
    digitalWrite(beeper_pin, LOW);

}

unsigned long currentMillis = 0;

void loop() {
    static int rc_counter = 0;

    while(Serial3.available()) {
        char c = Serial3.read();
        rxParseByte(c);
        rc_counter++;
        // Serial2.write(c);
    }

    if(millis() - currentMillis > 1000) {
        currentMillis = millis();
        Serial2.printf("RC: %d\n", rc_counter);
        rc_counter = 0;
    }

}
