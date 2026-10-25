#include "MatauliRobot.h"

// Initialize static members
const int MatauliRobot::LEFT_PINS[4] = {10, 11, 12, 13};
const int MatauliRobot::RIGHT_PINS[4] = {15, 16, 21, 33};
const int MatauliRobot::BUZZER_PIN = 14;
const int MatauliRobot::LASER_PIN = 40;
const int MatauliRobot::SERVO_PIN = 7;
const int MatauliRobot::TRIG_PIN = 38;
const int MatauliRobot::ECHO_PIN = 39;
const int MatauliRobot::IR_LED_PIN = 6;
const int MatauliRobot::SENSOR_PINS[5] = {1, 2, 3, 4, 5};
const int MatauliRobot::RGB_LED_PIN = 17;
const int MatauliRobot::NUM_LEDS = 3;

const float MatauliRobot::SPEED_OF_SOUND_CPS = 34800.0f;
const int MatauliRobot::TICK_US = 10;
const int MatauliRobot::TICKS_PER_MS = 100;

const int8_t MatauliRobot::STEP_SEQ[8][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

MatauliRobot* MatauliRobot::instance = nullptr;

// Constructor
MatauliRobot::MatauliRobot() 
    : leftStepIndex(0), rightStepIndex(0), leftDir(1), rightDir(1),
      buzzerCountdownMs(0), laserCountdownMs(0),
      tickCounter(0), msCounter(0), echoStartTick(0), 
      echoDurationTicks(0), echoHigh(false), measurementReady(false),
      rgbLeds(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800) {
    instance = this;
}

// Initialization
void MatauliRobot::begin() {
    setCpuFrequencyMhz(240);
    
    // Initialize motor pins
    for (int i = 0; i < 4; i++) {
        pinMode(LEFT_PINS[i], OUTPUT);
        pinMode(RIGHT_PINS[i], OUTPUT);
    }
    
    // Initialize active-low peripherals
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);
    
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, HIGH);
    
    // Initialize servo
    myServo.attach(SERVO_PIN);
    servo(0);
    
    // Initialize ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    
    // Initialize IR LED and sensors
    pinMode(IR_LED_PIN, OUTPUT);
    digitalWrite(IR_LED_PIN, LOW);
    
    for (int i = 0; i < 5; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }
    
    // Initialize RGB LEDs
    rgbLeds.begin();
    rgbLeds.clear();
    rgbLeds.show();
    
    // Stop motors
    wheel(0, 0);
    
    // Start 10us ticker
    msTicker.attach_us(TICK_US, msTickISR);
}

// ISR Callbacks
void MatauliRobot::stepLeftMotorISR() {
    if (!instance) return;
    
    instance->leftStepIndex = (instance->leftStepIndex + instance->leftDir + 8) % 8;
    for (int i = 0; i < 4; i++) {
        digitalWrite(LEFT_PINS[i], STEP_SEQ[instance->leftStepIndex][i]);
    }
}

void MatauliRobot::stepRightMotorISR() {
    if (!instance) return;
    
    instance->rightStepIndex = (instance->rightStepIndex + instance->rightDir + 8) % 8;
    for (int i = 0; i < 4; i++) {
        digitalWrite(RIGHT_PINS[i], STEP_SEQ[instance->rightStepIndex][i]);
    }
}

void MatauliRobot::msTickISR() {
    if (!instance) return;
    
    instance->tickCounter++;
    
    // Ultrasonic echo measurement
    bool echoState = digitalRead(ECHO_PIN);
    
    if (echoState && !instance->echoHigh) {
        instance->echoStartTick = instance->tickCounter;
        instance->echoHigh = true;
    } else if (!echoState && instance->echoHigh) {
        instance->echoDurationTicks = instance->tickCounter - instance->echoStartTick;
        instance->echoHigh = false;
        instance->measurementReady = true;
    }
    
    // Millisecond divider
    if ((instance->tickCounter % TICKS_PER_MS) == 0) {
        instance->msCounter++;
        
        if (instance->buzzerCountdownMs > 0) {
            instance->buzzerCountdownMs--;
            if (instance->buzzerCountdownMs == 0) {
                digitalWrite(BUZZER_PIN, HIGH);
            }
        }
        
        if (instance->laserCountdownMs > 0) {
            instance->laserCountdownMs--;
            if (instance->laserCountdownMs == 0) {
                digitalWrite(LASER_PIN, HIGH);
            }
        }
    }
}

// Motor control
void MatauliRobot::wheel(int leftSpeed, int rightSpeed) {
    if (leftSpeed == 0) {
        leftTicker.detach();
        for (int i = 0; i < 4; i++) digitalWrite(LEFT_PINS[i], LOW);
    } else {
        leftDir = (leftSpeed > 0) ? 1 : -1;
        leftTicker.attach(1.0f / abs(leftSpeed), stepLeftMotorISR);
    }
    
    if (rightSpeed == 0) {
        rightTicker.detach();
        for (int i = 0; i < 4; i++) digitalWrite(RIGHT_PINS[i], LOW);
    } else {
        rightDir = (rightSpeed > 0) ? -1 : 1;
        rightTicker.attach(1.0f / abs(rightSpeed), stepRightMotorISR);
    }
}

// Buzzer control
void MatauliRobot::buzzer(int duration) {
    if (duration <= 0) {
        buzzerCountdownMs = 0;
        digitalWrite(BUZZER_PIN, HIGH);
    } else {
        buzzerCountdownMs = duration;
        digitalWrite(BUZZER_PIN, LOW);
    }
}

// Laser control
void MatauliRobot::laser(int duration) {
    if (duration <= 0) {
        laserCountdownMs = 0;
        digitalWrite(LASER_PIN, HIGH);
    } else {
        laserCountdownMs = duration;
        digitalWrite(LASER_PIN, LOW);
    }
}

// Servo control
void MatauliRobot::servo(float angle) {
    if (angle < -90.0f) angle = -90.0f;
    if (angle > 90.0f) angle = 90.0f;
    myServo.write((int)(angle + 90.0f));
}

// Ultrasonic distance
float MatauliRobot::readDistance() {
    delay(60);
    
    measurementReady = false;
    echoHigh = false;
    
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    unsigned long timeoutMs = msCounter + 100;
    while (!measurementReady && (msCounter < timeoutMs)) {
        delay(1);
    }
    
    if (!measurementReady || echoDurationTicks == 0) {
        return -1.0;
    }
    
    float timeSeconds = (echoDurationTicks * (float)TICK_US) / 1000000.0f;
    float distance = (timeSeconds * SPEED_OF_SOUND_CPS) / 2.0f;
    
    return distance;
}

// Sensor reading
int MatauliRobot::readSensor(bool irLed) {
    digitalWrite(IR_LED_PIN, irLed ? HIGH : LOW);
    
    int result = 0;
    for (int i = 0; i < 5; i++) {
        if (digitalRead(SENSOR_PINS[i]) == HIGH) {
            result |= (1 << i);
        }
    }
    
    return result;
}

// RGB LED control
void MatauliRobot::rgbLed(int ledIndex, uint8_t r, uint8_t g, uint8_t b) {
    if (ledIndex < 0 || ledIndex >= NUM_LEDS) {
        return;
    }
    
    rgbLeds.setPixelColor(ledIndex, rgbLeds.Color(r, g, b));
    rgbLeds.show();
}
