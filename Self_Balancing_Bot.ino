#include <Wire.h>
#include <MPU6050_light.h>

// ---------------- MOTOR PINS ----------------
#define MOTOR_LEFT_PWM   12
#define MOTOR_LEFT_DIR   14

#define MOTOR_RIGHT_PWM  26
#define MOTOR_RIGHT_DIR  25

// ---------------- PID VARIABLES ----------------
float error = 0;
float previous_error = 0;

float Kp = 165;
float Ki = 0;
float Kd = 60;

float integral = 0;
float derivative = 0;

float output = 0;

// ---------------- TIME VARIABLES ----------------
unsigned long previous_time = 0;
unsigned long serial_timer = 0;

// ---------------- MPU OBJECT ----------------
MPU6050 mpu(Wire);

void setup() {

  // ---------------- MOTOR PIN SETUP ----------------
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);

  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);

  // ---------------- SERIAL COMMUNICATION ----------------
  Serial.begin(9600);

  // ---------------- MPU6050 INITIALIZATION ----------------
  Wire.begin();

  byte status = mpu.begin();

  Serial.print("MPU6050 Status: ");
  Serial.println(status);

  // Stop if MPU6050 not detected
  while (status != 0) {
  }

  Serial.println("Calculating MPU6050 offsets...");
  delay(1000);

  // Uncomment if sensor mounted upside down
  // mpu.upsideDownMounting = true;

  mpu.calcOffsets();

  Serial.println("MPU6050 Ready!");

  previous_time = micros();
}

void loop() {

  // ---------------- UPDATE MPU DATA ----------------
  mpu.update();

  // ---------------- PRINT ANGLE EVERY 10ms ----------------
  if ((millis() - serial_timer) > 10) {

    Serial.print("Angle Y: ");
    Serial.println(mpu.getAngleY());

    serial_timer = millis();
  }

  // ---------------- CURRENT ANGLE ----------------
  float current_angle = mpu.getAngleY();

  // ---------------- TIME DIFFERENCE ----------------
  unsigned long current_time = micros();

  float dt = (current_time - previous_time) / 1000000.0;

  previous_time = current_time;

  // ---------------- PID CALCULATIONS ----------------
  error = current_angle;

  integral += error * dt;

  derivative = (error - previous_error) / dt;

  output = (Kp * error) +
           (Ki * integral) +
           (Kd * derivative);

  previous_error = error;

  // ---------------- PWM LIMIT ----------------
  int pwm = constrain(abs(output), 0, 255);

  // ---------------- MOTOR CONTROL ----------------

  // Robot tilted backward
  if (current_angle < 0) {

    analogWrite(MOTOR_LEFT_PWM, pwm);
    digitalWrite(MOTOR_LEFT_DIR, LOW);

    analogWrite(MOTOR_RIGHT_PWM, pwm);
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  }

  // Robot tilted forward
  else {

    analogWrite(MOTOR_LEFT_PWM, pwm);
    digitalWrite(MOTOR_LEFT_DIR, HIGH);

    analogWrite(MOTOR_RIGHT_PWM, pwm);
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  }

  delay(10);
}