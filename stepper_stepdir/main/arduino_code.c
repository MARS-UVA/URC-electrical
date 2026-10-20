
#include <Arduino.h>
#include <Stepper.h>

// Number of steps per revolution
#define STEPS 200

// ESP32-S3 GPIO pins connected to the stepper driver
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

// Create stepper motor instance
Stepper stepper(STEPS, IN1, IN3, IN2, IN4);

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32-S3 Stepper Test!");

    // Set motor speed to 60 RPM
    stepper.setSpeed(60);
}

void loop()
{
    Serial.println("Forward");
    stepper.step(STEPS);

    delay(500);

    Serial.println("Backward");
    stepper.step(-STEPS);

    delay(500);
}
