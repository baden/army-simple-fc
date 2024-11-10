#include <Arduino.h>
#include <Servo.h>

// #include <Сrsf.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// Може це підійде?
// https://github.com/BobbyIndustries/crsfSerial

// Або це:
// https://docs.arduino.cc/libraries/alfredocrsf/#Releases

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

// Crsf crsf;
static HardwareSerial crsfSerial(USART3);
AlfredoCRSF crsf;

// static Crsf crsf(CrsfSerialStream);

// VTX UART is half-duplex
// https://github.com/AlanAlger001/ExpressLRS/blob/8e3248cc0bfe9b015bf527616cd7259d88ce0f58/src/src/rx_main.cpp#L135
HardwareSerial VtxSerial(USART4, HALF_DUPLEX_ENABLED);
// HardwareSerial CrsfRxSerial(USART1, HALF_DUPLEX_ENABLED);

enum beeper_state {
    BEEPER_STATE_OFF,
    BEEPER_STATE_SHORT_SIGNAL,
    BEEPER_STATE_SHORT_PAUSE,
    // BEEPER_STATE_LONG_SIGNAL,
    // BEEPER_STATE_LONG_PAUSE,
};

#define BEEPER_SHORT_SIGNAL_DURATION 100
#define BEEPER_SHORT_PAUSE_DURATION 300
#define BEEPER_LONG_SIGNAL_DURATION 500
#define BEEPER_LONG_PAUSE_DURATION 500

#define TASK_PERIOD_MS 100

unsigned beeper_signals = 0;
unsigned beeper_state = BEEPER_STATE_OFF;
int beeper_timer = 0;

// 100ms beeper task
void beeper_task() {
    switch (beeper_state)
    {
    case BEEPER_STATE_OFF:
        digitalWrite(beeper_pin, LOW);
        break;
    case BEEPER_STATE_SHORT_SIGNAL:
        if(beeper_timer <= 0) {
            beeper_state = BEEPER_STATE_SHORT_PAUSE;
            digitalWrite(beeper_pin, LOW);
            beeper_timer = BEEPER_SHORT_PAUSE_DURATION;
        } else {
            digitalWrite(beeper_pin, HIGH);
            beeper_timer -= TASK_PERIOD_MS;
        }
        break;
    case BEEPER_STATE_SHORT_PAUSE:
        if(beeper_timer <= 0) {
            beeper_signals--;
            if(beeper_signals == 0) {
                beeper_state = BEEPER_STATE_OFF;
                digitalWrite(beeper_pin, LOW);
            } else {
                beeper_state = BEEPER_STATE_SHORT_SIGNAL;
                digitalWrite(beeper_pin, HIGH);
                beeper_timer = BEEPER_SHORT_SIGNAL_DURATION;
            }
        } else {
            digitalWrite(beeper_pin, LOW);
            beeper_timer -= TASK_PERIOD_MS;
        }
        break;
    
    default:
        break;
    }
}

void beeper_signal(unsigned signals) {
    beeper_signals = signals;
    beeper_state = BEEPER_STATE_SHORT_SIGNAL;
    beeper_timer = BEEPER_SHORT_SIGNAL_DURATION;
}

void setup() {
    // put your setup code here, to run once:
    Serial2.begin(115200);
    Serial2.printf("Booting...\n");

    // Init RF UART
    // Serial3.begin(420000);

    // Init RX UART
    crsfSerial.begin(CRSF_BAUDRATE);
    crsf.begin(crsfSerial);

    // Init motors servos and set to 0
    motor1_servo.attach(moto1_servo_pin);
    motor1_servo.write(90);
    motor2_servo.attach(moto2_servo_pin);
    motor2_servo.write(90);
    motor3_servo.attach(moto3_servo_pin);
    motor3_servo.write(90);
    motor4_servo.attach(moto4_servo_pin);
    motor4_servo.write(90);

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

    beeper_signal(1);       // При подачі живлення сигнал один раз
}

unsigned long currentMillis = 0;

#define RC_TIMEOUT_MS 3000
#define RC_DRIVER_TIMEOUT_MS 60000

void loop() {
    static int rc_counter = 0;
    static bool rc_link = false;
    static int rc_timeout = 0;
    static bool rc_timeout_flag = true;
    static int rc_driver_timeout = 0;
    static bool rc_driver_timeout_flag = true;

    crsf.update();
    // while(Serial3.available()) {
    //     char c = Serial3.read();
    //     rxParseByte(c);
    //     rc_counter++;
    //     // Serial2.write(c);
    // }

    if(millis() - currentMillis > TASK_PERIOD_MS) {
        currentMillis = millis();
        // Serial2.printf("RC: %d\n", rc_counter);
        // rc_counter = 0;

        if(crsf.isLinkUp()) {

            if(!rc_link) {
                // Serial2.println("Link is up");
                rc_link = true;
                // Два коротких сигнала
                beeper_signal(2);
            }

            rc_timeout = RC_TIMEOUT_MS;
            rc_timeout_flag = false;
            rc_driver_timeout = RC_DRIVER_TIMEOUT_MS;
            rc_driver_timeout_flag = false;

            // Канал 5 - Arm/Disarm
            // Вимикає:
            // - стартер
            // - лебідку
            // - мотори
            // Працює:
            // - декомпресор
            // - газування двигуна

            bool armed = crsf.getChannel(5) > 1500;
            // TODO: Зробити довгий звуковий сигнал

            // Канал 3 - серво для керування газом двигуна
            int acc_value = crsf.getChannel(3);
            int servoPos = map(acc_value, 1000, 2000, 0, 180);
            acc_servo.write(servoPos);

            // Канал 11 - дозвіл на керування двигуном (не задіян)

            // Канал 10 - Вихід стартера
            int starter_value = crsf.getChannel(10);
            if(starter_value > 1500) {
                if(armed) {
                    digitalWrite(starter_pin, HIGH);
                } else {
                    digitalWrite(starter_pin, LOW);
                }
            } else {
                digitalWrite(starter_pin, LOW);
            }

            // Канал 9 - Вихід декомпресора
            int decompressor_value = crsf.getChannel(9);
            if(decompressor_value > 1500) {
                digitalWrite(decompressor_pin, HIGH);
            } else {
                digitalWrite(decompressor_pin, LOW);
            }


            // Канал 12 - Керування лебідкою
            int winch_value = crsf.getChannel(12);
            if(armed) {
                if(winch_value > 1750) {
                    digitalWrite(winch_up_pin, HIGH);
                    digitalWrite(winch_down_pin, LOW);
                } else if(winch_value < 1250) {
                    digitalWrite(winch_up_pin, LOW);
                    digitalWrite(winch_down_pin, HIGH);
                } else {
                    digitalWrite(winch_up_pin, LOW);
                    digitalWrite(winch_down_pin, LOW);
                }
            } else {
                digitalWrite(winch_up_pin, LOW);
                digitalWrite(winch_down_pin, LOW);
            }

            #define CENTER_VALUE 1500

            // Керування моторами
            // Канал 2 - Швидкість
            // Канал 1 - Кут руху
            int throttle_value = crsf.getChannel(2) - CENTER_VALUE;
            int angle_value = crsf.getChannel(1) - CENTER_VALUE;

            // Мотори 1 та 2 - лівий борт
            // Мотори 3 та 4 - правий борт
            int motor_left_speed = throttle_value + angle_value;
            int motor_right_speed = throttle_value - angle_value;

            // Map to 0-180
            int motor1_pos = map(motor_left_speed, -500, 500, 0, 180);
            int motor2_pos = map(motor_left_speed, -500, 500, 0, 180);
            int motor3_pos = map(motor_right_speed, -500, 500, 0, 180);
            int motor4_pos = map(motor_right_speed, -500, 500, 0, 180);

            if(armed) {
                motor1_servo.write(motor1_pos);
                motor2_servo.write(motor2_pos);
                motor3_servo.write(motor3_pos);
                motor4_servo.write(motor4_pos);
            } else {
                motor1_servo.write(90);
                motor2_servo.write(90);
                motor3_servo.write(90);
                motor4_servo.write(90);
            }

            // Serial2.print(motor1_pos);
            // Serial2.print(", ");
            // Serial2.println(motor3_pos);


        } else {
            if(rc_link) {
                // Serial2.println("Link is down");
                rc_link = false;
            }

            // Вимкнути небезпечні функції (треба зробити затримку)
            if(rc_timeout > 0) {
                rc_timeout -= TASK_PERIOD_MS;
            } else {
                if(!rc_timeout_flag) {
                    rc_timeout_flag = true;

                    // Три сигнали
                    beeper_signal(3);

                }
                digitalWrite(starter_pin, LOW);
                digitalWrite(decompressor_pin, LOW);
                digitalWrite(winch_up_pin, LOW);
                digitalWrite(winch_down_pin, LOW);
                // TODO: Ще через пару (?) хвилин треба зупинити двигун
                // Зупинити мотори (в середнє положення)
                motor1_servo.write(90);
                motor2_servo.write(90);
                motor3_servo.write(90);
                motor4_servo.write(90);
            }

            if(rc_driver_timeout > 0) {
                rc_driver_timeout -= TASK_PERIOD_MS;
            } else {
                if(!rc_driver_timeout_flag) {
                    rc_driver_timeout_flag = true;
                    // Скинути газ на мінімум (бажано б якось перевіряти шо двигун працює, а то це буде спрацьовувати завжди)
                    acc_servo.write(0);
                    delay(3000);
                    // Видати сигнал 1секунду на декомпресор
                    digitalWrite(decompressor_pin, HIGH);
                    // Затримка 1 секунда
                    delay(1000);
                    digitalWrite(decompressor_pin, LOW);
                    // 4 звукових сигнали
                    beeper_signal(4);
                }
            }
        }

        beeper_task();

        #if 0
        if(crsf.isLinkUp()) {
            Serial2.println("Link is up");
            // Serial2.print("Channels: ");
        } else {
            Serial2.println("Link is down");
        }

        for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++) {
            Serial2.print(crsf.getChannel(ChannelNum));
            Serial2.print(", ");
        }
        Serial2.println(" ");
        #endif

        static int simulate_battery_voltage = 40;
        static int simulate_battery_remaining = 25;
        crsf_sensor_battery_t crsfBatt = { 0 };

        crsfBatt.voltage = htobe16((uint16_t)(simulate_battery_voltage * 10.0));   //Volts
        crsfBatt.current = htobe16(10);  //Amps
        crsfBatt.capacity = htobe16(1000);   //mAh
        crsfBatt.remaining = simulate_battery_remaining; //Percent
        crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));

        simulate_battery_remaining = 100 - simulate_battery_remaining;



    }

}
