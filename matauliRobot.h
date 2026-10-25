#ifndef MATAULIROBOT_H
#define MATAULIROBOT_H

#include <Arduino.h>
#include <Ticker.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

class MatauliRobot {
public:
    // Constructor
    MatauliRobot();
    
    // Initialization
    void begin();
    
    // Motor control
    void wheel(int leftSpeed, int rightSpeed);
    
    // Peripheral control
    void buzzer(int duration);
    void laser(int duration);
    void servo(float angle);
    
    // Sensor reading
    float readDistance();
    int readSensor(bool irLed);
    
    // RGB LED control
    void rgbLed(int ledIndex, uint8_t r, uint8_t g, uint8_t b);

private:
    // Pin definitions
    static const int LEFT_PINS[4];
    static const int RIGHT_PINS[4];
    static const int BUZZER_PIN;
    static const int LASER_PIN;
    static const int SERVO_PIN;
    static const int TRIG_PIN;
    static const int ECHO_PIN;
    static const int IR_LED_PIN;
    static const int SENSOR_PINS[5];
    static const int RGB_LED_PIN;
    static const int NUM_LEDS;
    
    // Constants
    static const float SPEED_OF_SOUND_CPS;
    static const int TICK_US;
    static const int TICKS_PER_MS;
    
    // Motor state
    static const int8_t STEP_SEQ[8][4];
    int leftStepIndex;
    int rightStepIndex;
    int leftDir;
    int rightDir;
    
    // Peripheral state
    volatile int buzzerCountdownMs;
    volatile int laserCountdownMs;
    
    // Ultrasonic state
    volatile unsigned long tickCounter;
    volatile unsigned long msCounter;
    volatile unsigned long echoStartTick;
    volatile unsigned long echoDurationTicks;
    volatile bool echoHigh;
    volatile bool measurementReady;
    
    // Tickers
    Ticker leftTicker;
    Ticker rightTicker;
    Ticker msTicker;
    
    // Servo
    Servo myServo;
    
    // RGB LEDs
    Adafruit_NeoPixel rgbLeds;
    
    // ISR callbacks (must be static)
    static void stepLeftMotorISR();
    static void stepRightMotorISR();
    static void msTickISR();
    
    // Static instance pointer for ISR access
    static MatauliRobot* instance;
};

#endif // MATAULIROBOT_H
