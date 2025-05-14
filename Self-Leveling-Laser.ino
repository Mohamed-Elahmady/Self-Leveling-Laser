#include <Servo.h>
#include "MPU_KF.h"

#define LZR 2      // ليزر على PIN 2
#define SVM 5      // سيرفو على PIN 5

Servo myServo;

void setup() {
  Serial.begin(115200);

  pinMode(LZR, OUTPUT);
  IMU::init();
  myServo.attach(SVM);

  myServo.write(90); // وضعية بداية متوسطة
}

void loop() {
  digitalWrite(LZR, HIGH);  // تشغيل الليزر
  IMU::read();              // قراءة بيانات المستشعر

  double pitch = IMU::getPitch(); // زاوية الميلان حول محور X
  static double previousError = 0.0;
  static double integral = 0.0;
  static double currentAngle = 90.0;

  // ثوابت الكنترولر PID
  const double kp = 1.2;
  const double ki = 0.01;
  const double kd = 0.3;
  const double deadband = 0.5;

  // خليه يميل مع نفس زاوية الـ pitch
  double targetAngle = constrain(90.0 + pitch, 0, 180);  // ممكن تستخدم +pitch لو عايزه الاتجاه العكسي

  // التحكم باستخدام PID
  double error = targetAngle - currentAngle;
  if (abs(error) > deadband) {
    integral += error;
    double derivative = error - previousError;
    previousError = error;

    double output = kp * error + ki * integral + kd * derivative;
    currentAngle = constrain(currentAngle + output, 0, 180);
    myServo.write(currentAngle);
  }

  // طباعة القيم للمراقبة
  Serial.println("------------------------------------------------------------");
  Serial.print("Roll: "); Serial.print(IMU::getRoll(), 2);
  Serial.print("\tPitch: "); Serial.print(pitch, 2);
  Serial.print("\tServo Angle: "); Serial.println(currentAngle, 2);
  Serial.println("------------------------------------------------------------");

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