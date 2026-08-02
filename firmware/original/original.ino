#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;

// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// Variables for complementary filter
float alpha = 0.95;     // Adjust alpha to give more weight to accelerometer  
float angle = 0.0;      // Computed angle of inclination
float gyroAngle = 0.0;  // Gyro-derived angle
float accAngle = 0.0;   // Accelerometer-derived angle
unsigned long prevTime = 0;
unsigned long gyroResetTime = 0;
unsigned long gyroResetInterval = 1000; // Adjust this interval as needed

// Variables for PID control
double setpoint = 2.0;  // Desired angle (adjust as needed)
double prevError = 0.0;
double integral = 0.0;
double Kp = 100.0;       // Proportional term
double Ki = 1.0;         // Integral term (adjust as needed)
double Kd = 5.0;        // Derivative term (adjust as needed)
double maxIntegral = 100.0; // Maximum integral value to prevent windup

// Encoder feedback (if available)
// int encoderA = 2;     // Connect encoder pins to your Arduino pins
// int encoderB = 3;

// Safety parameters
double maxMotorOutput = 255.0; // Maximum motor speed
double minMotorOutput = 40.0;  // Minimum motor speed to prevent stall
double maxAngleError = 15.0;   // Maximum allowable angle error before safety stops
unsigned long safetyInterval = 500; // Safety check interval

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  mpu.setup(0x68); // Set the correct address for your sensor

  // Motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  if (mpu.update()) {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Calculate time interval in seconds

    // Calculate accelerometer-derived angle
    float accY = mpu.getAccY();
    float accZ = mpu.getAccZ();
    accAngle = atan2(accY, accZ) * RAD_TO_DEG;

    // Calculate gyro-derived angle change
    float gyroX = mpu.getGyroX();
    gyroAngle += gyroX * dt;

    // Apply gyro drift correction
    if ((currentTime - gyroResetTime) >= gyroResetInterval) {
      gyroAngle = accAngle; // Reset gyro-derived angle to match accelerometer
      gyroResetTime = currentTime;
    }

    // Combine gyro and accelerometer angles using a complementary filter
    angle = alpha * gyroAngle + (1.0 - alpha) * accAngle;

    // Calculate PID error
    double error = setpoint - angle;
    integral += error * dt;
    integral = constrain(integral, -maxIntegral, maxIntegral); // Anti-windup
    double derivative = (error - prevError) / dt;

    // Calculate PID control output
    double output = Kp * error + Ki * integral + Kd * derivative;

    // Safety check to prevent excessive motor speed
    if (abs(error) > maxAngleError) {
      output = 0.0; // Emergency stop if angle error exceeds limit
    } else {
      output = constrain(output, -maxMotorOutput, maxMotorOutput);
      if (abs(output) < minMotorOutput) {
        // Prevent low-speed stall by maintaining a minimum motor speed
        output = (output > 0) ? minMotorOutput : -minMotorOutput;
      }
    }

    // Motor control based on PID output
    if (output > 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    } else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }

    // Adjust motor speed based on PID output
    analogWrite(enA, abs(output));
    analogWrite(enB, abs(output));

    prevError = error;
    prevTime = currentTime;

    // Print debug information
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" | PID Output: ");
    Serial.println(output);
  }
}