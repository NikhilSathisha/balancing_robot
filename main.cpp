#include "mbed.h"          // Core Mbed OS functionality
#include "Motor.h"         // Motor control library
#include "AnalogIn.h"      // Analog input library (if needed)
#include "ICM20948.h"      // IMU (Inertial Measurement Unit) library
#include "uLCD.hpp"        // uLCD display library
#include <chrono>          // Chrono library for time-related functions
#include <cmath>           // Math library for mathematical functions
#include <cstdlib>         // C standard library for general utilities

using namespace std::chrono_literals; // Enable chrono literals (e.g., 10ms)

// Pi constant
constexpr float PI = 3.14159265358979323846f;

// Initialize peripherals and components
ICM20948 imu(p28, p27);     // IMU on pins p28 (SDA) and p27 (SCL)
Motor m1(p23, p6, p5);      // Motor 1 with enable, forward, and reverse pins
Motor m2(p26, p7, p8);      // Motor 2 with enable, forward, and reverse pins
uLCD display(p9, p10, p11, uLCD::BAUD_9600); // uLCD display on specified pins
Thread lcdThread;           // Thread for LCD display updates
PwmOut speaker(p21);        // PWM output for speaker on pin p21

// PID Control Parameters
float Kp = 2.75f;           // Proportional gain
float Ki = 0.0f;            // Integral gain
float Kd = 0.1f;            // Derivative gain

float targetAz = -0.10f;    // Target azimuth angle for balance
float prevError = 0.0f;     // Previous error for derivative calculation
float integral = 0.0f;      // Integral sum for PID control

volatile float current_error = 0.0f; // Current error shared across threads
const float dt = 0.01f; // PID loop interval (10ms)

EventQueue queue; // Event queue for managing callbacks
Ticker controlTicker; // Ticker to schedule periodic PID updates

// LCD Display Function
// Updates the display with a happy or sad face based on error magnitude
// Emits sound via speaker if the robot is tilted (|error| >= 0.6)
void lcd_face() {
    printf("LCD thread started\n"); // Debug message for thread initialization
    display.cls(); // Clear the display
    speaker.period(1.0f / 500.0f); // Set speaker frequency to 500Hz

    bool last_face_happy = false; // Tracks previous face state
    bool first_draw = true;       // Ensures the first face is always drawn

    while (true) {
        bool face_happy = (fabs(current_error) < 0.6f); // Determine face state

        // Manage speaker based on tilt state
        if (face_happy) {
            speaker = 0; // No sound for upright state
            printf("no noise\n");
        } else {
            speaker = 0.5f; // Emit sound for tilted state
            printf("noise\n");
        }

        // Update the face only if the state has changed or it's the first draw
        if (first_draw || (face_happy != last_face_happy)) {
            printf("Face state changed: %s\n", face_happy ? "Happy" : "Sad");
            display.cls(); // Clear display before drawing new face

            int centerX = 64; // Face center X-coordinate
            int centerY = 64; // Face center Y-coordinate
            int radius = 30;  // Face radius

            display.drawCircleFilled(centerX, centerY, radius, 0xFFFF); // Draw head
            display.drawCircleFilled(centerX - 10, centerY + 12, 3, 0x0000); // Left eye
            display.drawCircleFilled(centerX + 10, centerY + 12, 3, 0x0000); // Right eye

            // Draw appropriate mouth
            if (face_happy) {
                // Draw smile (downward arc)
                for (int angle = 180; angle <= 360; angle += 15) {
                    float rad1 = angle * PI / 180.0f;
                    float rad2 = (angle + 15) * PI / 180.0f;
                    int x1 = centerX + 20 * cos(rad1);
                    int y1 = centerY + 10 * sin(rad1);
                    int x2 = centerX + 20 * cos(rad2);
                    int y2 = centerY + 10 * sin(rad2);
                    display.drawLine(x1, y1, x2, y2, 0x0000);
                }
            } else {
                // Draw frown (upward arc)
                for (int angle = 0; angle <= 180; angle += 15) {
                    float rad1 = angle * PI / 180.0f;
                    float rad2 = (angle + 15) * PI / 180.0f;
                    int x1 = centerX + 20 * cos(rad1);
                    int y1 = centerY + 10 * sin(rad1) - 15;
                    int x2 = centerX + 20 * cos(rad2);
                    int y2 = centerY + 10 * sin(rad2) - 15;
                    display.drawLine(x1, y1, x2, y2, 0x0000);
                }
            }

            last_face_happy = face_happy; // Update last face state
            first_draw = false;          // Mark that the first draw is done
        }

        // Display current error on the uLCD
        display.locate(0, 0);
        display.printf("Error: %.2f  ", current_error);

        ThisThread::sleep_for(200ms); // Update rate to reduce flicker
    }
}

// PID Control Function
// Reads IMU data, calculates PID output, and adjusts motor speeds
void updateControl() {
    imu.getAccGyro(); // Read accelerometer and gyroscope data
    float az = imu.getAZ(); // Get current azimuth angle

    float error = az - targetAz; // Calculate error
    integral += error * dt;     // Update integral term
    float derivative = (error - prevError); // Calculate derivative term
    prevError = error;          // Store current error for next iteration

    current_error = error;      // Update shared error variable

    float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative); // PID output

    if (pidOutput > 1.0f) pidOutput = 1.0f;
    if (pidOutput < -1.0f) pidOutput = -1.0f;

    if (fabs(error) > 0.85f) { // Stop motors if tilt exceeds threshold
        m1.speed(0);
        m2.speed(0);
    } else {
        m1.speed(-pidOutput); // Adjust motor speeds based on PID output
        m2.speed(-pidOutput);
    }

    printf("az=%.2f, error=%.2f, pidOutput=%.2f\n", az, error, pidOutput); // Debug output
}

// Event Queue Callback
// Enqueues the PID control function into the event queue
void postUpdateControl() {
    queue.call(updateControl);
}

int main() {
    printf("1: Initializing IMU\n");
    imu.init((1 | (1 << 1) | (0 << 3)), 1, (1 | (1 << 1) | (0 << 3)), 1); // Initialize IMU
    printf("2: IMU Initialized\n");

    controlTicker.attach(&postUpdateControl, 10ms); // Schedule PID updates
    printf("3: Control Loop Started\n");

    lcdThread.start(lcd_face); // Start LCD thread
    printf("4: LCD Thread Started\n");

    queue.dispatch_forever(); // Process events indefinitely
}