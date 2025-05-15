#include <Servo.h>
#include "MPU_KF.h"

#define LZR 6      // Laser on PIN 6
#define SVM 5      // Servo on PIN 5

Servo myServo;

void setup() {
  Serial.begin(115200);

  pinMode(LZR, OUTPUT);
  IMU::init();
  myServo.attach(SVM);

  myServo.write(92.20); // Initial mid-position
}

void loop() {
  digitalWrite(LZR, HIGH);

  IMU::read();  // Read MPU values
  double rawPitch = IMU::getPitch();

  // --- Calibrate pitch angle ---
  const double pitchOffset = 9.2921711;  // Compensation for MPU angle deviation
  double pitch = rawPitch + pitchOffset;

  // --- PID Constants ---
  static double currentAngle = 92.20;  // Initial servo position
  static double previousError = 0.0;
  static double integral = 0.0;

  const double kp = 1.2;
  const double ki = 0.01;
  const double kd = 0.3;
  const double deadband = 0.5;

  // --- Target angle for the servo ---
  double targetAngle = constrain(90.0 + pitch, 0, 180);

  // --- Compute error ---
  double error = targetAngle - currentAngle;

  // --- Apply PID only if error is outside deadband ---
  if (abs(error) > deadband) {
    integral += error;
    integral = constrain(integral, -50, 50);  // Limit integral wind-up

    double derivative = error - previousError;
    previousError = error;

    double output = kp * error + ki * integral + kd * derivative;

    currentAngle += output;
    currentAngle = constrain(currentAngle, 0, 180);

    myServo.write(currentAngle);
  }

  // --- Monitor values ---
  Serial.print("Raw Pitch: "); Serial.print(rawPitch, 2);
  Serial.print(" | Calibrated Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Servo Angle: "); Serial.println(currentAngle, 2);

  delay(50);
}


  // Serial.println("------------------------------------------------------------");
  // Serial.println("Raw Sensor Data:");
  // Serial.print("Accel [X, Y, Z]: ");
  // Serial.print(IMU::getRawAccelX()); Serial.print("\t");
  // Serial.print(IMU::getRawAccelY()); Serial.print("\t");
  // Serial.println(IMU::getRawAccelZ());

  // Serial.print("Gyro  [X, Y, Z]: ");
  // Serial.print(IMU::getRawGyroX()); Serial.print("\t");
  // Serial.print(IMU::getRawGyroY()); Serial.print("\t");
  // Serial.println(IMU::getRawGyroZ());

  // Serial.println("------------------------------------------------------------");
  // Serial.println("Processed Sensor Data:");
  // Serial.print("Accel [X, Y, Z] (g): ");
  // Serial.print(IMU::getAccelX_g(), 4); Serial.print("\t");
  // Serial.print(IMU::getAccelY_g(), 4); Serial.print("\t");
  // Serial.println(IMU::getAccelZ_g(), 4);

  // Serial.print("Gyro  [X, Y, Z] (°/s): ");
  // Serial.print(IMU::getGyroX_dps(), 2); Serial.print("\t");
  // Serial.print(IMU::getGyroY_dps(), 2); Serial.print("\t");
  // Serial.println(IMU::getGyroZ_dps(), 2);

  // Serial.println("------------------------------------------------------------");
  // Serial.print("Roll: "); Serial.print(IMU::getRoll(), 2);
  // Serial.print("\tPitch: "); Serial.println(IMU::getPitch(), 2);
  // Serial.println("------------------------------------------------------------");

  // delay(1000);



// void loop() {
//   digitalWrite(LZR, HIGH);
//   IMU::read();

//   double roll = IMU::getRoll();
//   static bool tilted = false;

//   if (abs(roll) > 4.0 && !tilted) {
//     myServo.write(roll > 0 ? 60 : 120);
//     tilted = true;
//     Serial.print("Tilted! Roll = "); Serial.println(roll);
//   } 
//   else if (abs(roll) <= 4.0 && tilted) {
//     myServo.write(90);
//     tilted = false;
//     Serial.print("Back to level. Roll = "); Serial.println(roll);
//   }

//   delay(50);
// }