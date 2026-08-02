#include <Wire.h>
#include "MPU9250.h"

namespace Config {
constexpr uint8_t EN_A = 9;
constexpr uint8_t IN_1 = 8;
constexpr uint8_t IN_2 = 7;
constexpr uint8_t EN_B = 3;
constexpr uint8_t IN_3 = 5;
constexpr uint8_t IN_4 = 4;
constexpr uint8_t IMU_ADDRESS = 0x68;
constexpr uint32_t LOOP_US = 10000;
constexpr float SETPOINT_DEG = 2.0f;
constexpr float ALPHA = 0.95f;
constexpr float KP = 100.0f;
constexpr float KI = 1.0f;
constexpr float KD = 5.0f;
constexpr float INTEGRAL_LIMIT = 100.0f;
constexpr float PWM_LIMIT = 255.0f;
constexpr float MINIMUM_PWM = 40.0f;
constexpr float ERROR_DEADBAND_DEG = 0.30f;
constexpr float ARM_WINDOW_DEG = 5.0f;
constexpr float FAULT_ANGLE_DEG = 15.0f;
}

enum class ControlState : uint8_t { DISARMED, BALANCING, FAULT };

MPU9250 imu;
ControlState state = ControlState::DISARMED;
uint32_t previousLoopUs = 0;
uint32_t bootMs = 0;
float filteredAngle = 0.0f;
float previousAngle = 0.0f;
float integral = 0.0f;

const char *stateName() {
  switch (state) {
    case ControlState::DISARMED: return "DISARMED";
    case ControlState::BALANCING: return "BALANCING";
    default: return "FAULT";
  }
}

void stopMotors() {
  analogWrite(Config::EN_A, 0);
  analogWrite(Config::EN_B, 0);
}

void driveMotors(float command) {
  if (command == 0.0f) {
    stopMotors();
    return;
  }
  const bool forward = command > 0.0f;
  digitalWrite(Config::IN_1, forward ? LOW : HIGH);
  digitalWrite(Config::IN_2, forward ? HIGH : LOW);
  digitalWrite(Config::IN_3, forward ? LOW : HIGH);
  digitalWrite(Config::IN_4, forward ? HIGH : LOW);
  const int pwm = constrain(lround(abs(command)), 0, 255);
  analogWrite(Config::EN_A, pwm);
  analogWrite(Config::EN_B, pwm);
}

void resetController(float angle) {
  integral = 0.0f;
  filteredAngle = angle;
  previousAngle = angle;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(Config::EN_A, OUTPUT);
  pinMode(Config::IN_1, OUTPUT);
  pinMode(Config::IN_2, OUTPUT);
  pinMode(Config::EN_B, OUTPUT);
  pinMode(Config::IN_3, OUTPUT);
  pinMode(Config::IN_4, OUTPUT);
  stopMotors();

  delay(2000);
  if (!imu.setup(Config::IMU_ADDRESS)) {
    state = ControlState::FAULT;
    Serial.println("IMU_SETUP_FAILED");
    return;
  }
  Serial.println("Keep the robot stationary: calibrating accelerometer and gyro.");
  delay(3000);
  imu.calibrateAccelGyro();
  bootMs = millis();
  previousLoopUs = micros();
  Serial.println("time_ms,dt_s,acc_angle_deg,gyro_rate_dps,fused_angle_deg,error_deg,p_term,i_term,d_term,command,state");
}

void loop() {
  if (state == ControlState::FAULT && Serial.available() && Serial.read() == 'r') {
    if (imu.update()) {
      const float angle = atan2(imu.getAccY(), imu.getAccZ()) * RAD_TO_DEG;
      if (abs(Config::SETPOINT_DEG - angle) <= Config::ARM_WINDOW_DEG) {
        resetController(angle);
        state = ControlState::DISARMED;
        bootMs = millis();
      }
    }
  }

  const uint32_t nowUs = micros();
  if (nowUs - previousLoopUs < Config::LOOP_US) return;
  const float dt = (nowUs - previousLoopUs) / 1000000.0f;
  previousLoopUs = nowUs;

  if (dt < 0.001f || dt > 0.05f || !imu.update()) {
    state = ControlState::FAULT;
    stopMotors();
    return;
  }

  const float accAngle = atan2(imu.getAccY(), imu.getAccZ()) * RAD_TO_DEG;
  const float gyroRate = imu.getGyroX();
  filteredAngle = Config::ALPHA * (filteredAngle + gyroRate * dt) +
                  (1.0f - Config::ALPHA) * accAngle;
  const float error = Config::SETPOINT_DEG - filteredAngle;

  if (state == ControlState::DISARMED) {
    stopMotors();
    resetController(filteredAngle);
    if (millis() - bootMs > 2000 && abs(error) <= Config::ARM_WINDOW_DEG) {
      state = ControlState::BALANCING;
    }
    return;
  }

  if (abs(error) > Config::FAULT_ANGLE_DEG) {
    state = ControlState::FAULT;
    stopMotors();
    return;
  }

  const float derivative = -(filteredAngle - previousAngle) / dt;
  const float candidateIntegral = constrain(
      integral + error * dt, -Config::INTEGRAL_LIMIT, Config::INTEGRAL_LIMIT);
  const float candidateOutput = Config::KP * error + Config::KI * candidateIntegral +
                                Config::KD * derivative;
  const float saturatedCandidate = constrain(
      candidateOutput, -Config::PWM_LIMIT, Config::PWM_LIMIT);
  if (candidateOutput == saturatedCandidate || error * candidateOutput < 0.0f) {
    integral = candidateIntegral;
  }

  const float pTerm = Config::KP * error;
  const float iTerm = Config::KI * integral;
  const float dTerm = Config::KD * derivative;
  float command = constrain(pTerm + iTerm + dTerm, -Config::PWM_LIMIT, Config::PWM_LIMIT);
  if (abs(error) <= Config::ERROR_DEADBAND_DEG) {
    command = 0.0f;
  } else if (abs(command) < Config::MINIMUM_PWM) {
    command = command > 0.0f ? Config::MINIMUM_PWM : -Config::MINIMUM_PWM;
  }

  driveMotors(command);
  previousAngle = filteredAngle;

  Serial.print(millis()); Serial.print(',');
  Serial.print(dt, 5); Serial.print(',');
  Serial.print(accAngle, 3); Serial.print(',');
  Serial.print(gyroRate, 3); Serial.print(',');
  Serial.print(filteredAngle, 3); Serial.print(',');
  Serial.print(error, 3); Serial.print(',');
  Serial.print(pTerm, 3); Serial.print(',');
  Serial.print(iTerm, 3); Serial.print(',');
  Serial.print(dTerm, 3); Serial.print(',');
  Serial.print(command, 1); Serial.print(',');
  Serial.println(stateName());
}
