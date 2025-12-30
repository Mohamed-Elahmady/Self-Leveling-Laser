#include "MPU_KF.h"
#include <Servo.h>
#include <SoftwareSerial.h>

// ================== PINS ==================
#define LZR PB3
#define SVM PA8

#define LED_G1 PB4
#define LED_G2 PB5
#define LED_R1 PB6
#define LED_R2 PB7

#define BT_RX PA10 // Bluetooth RX
#define BT_TX PA9  // Bluetooth TX

Servo myServo;
SoftwareSerial BT(BT_RX, BT_TX);

// =======================================================
// GLOBAL Bluetooth Angle
// =======================================================
double BT_angle = 23; // default base angle
const double BASE_ANGLE = 23;

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  BT.begin(9600);

  pinMode(LZR, OUTPUT);
  pinMode(LED_G1, OUTPUT);
  pinMode(LED_G2, OUTPUT);
  pinMode(LED_R1, OUTPUT);
  pinMode(LED_R2, OUTPUT);

  IMU::init();
  myServo.attach(SVM);

  myServo.write(92);

  digitalWrite(LED_G1, HIGH);
  digitalWrite(LED_G2, HIGH);
}

// =======================================================
// LOOP
// =======================================================
void loop() {

  // Read Bluetooth angle
  Bluetooth_Read_Angle();

  digitalWrite(LZR, HIGH);

  // Send BT angle to system
  Self_Laser_System(BT_angle);

  delay(50);
}

// =======================================================
// READ ANGLE FROM BLUETOOTH
// =======================================================
void Bluetooth_Read_Angle() {

  if (BT.available()) {
    String data = BT.readStringUntil('\n');
    data.trim();

    if (data.length() > 0) {
      double mobileAngle = data.toDouble();

      // Case 1 → mobile sends 0, 45, -45
      if (mobileAngle == 0 || mobileAngle == 45 || mobileAngle == -45) {
        BT_angle = BASE_ANGLE + mobileAngle;
      }
      // Case 2 → mobile sends -90 → +90
      else if (mobileAngle >= -90 && mobileAngle <= 90) {
        BT_angle = BASE_ANGLE + mobileAngle;
      }

      Serial.print("[BT] Received angle: ");
      Serial.println(BT_angle);
    }
  }
}

// =======================================================
// FUNCTION: MPU + LEDs + SERVO
// =======================================================
void Self_Laser_System(double externalOffset) {
  IMU::read();
  double rawPitch = IMU::getPitch();

  // Pitch after BT offset
  double pitch = rawPitch + externalOffset;

  // LED CONTROL
  const double tiltThreshold = 5.0;
  double pitchForLED = pitch - 16;

  if (abs(pitchForLED) > tiltThreshold) {
    digitalWrite(LED_G1, LOW);
    digitalWrite(LED_G2, LOW);
    digitalWrite(LED_R1, HIGH);
    digitalWrite(LED_R2, HIGH);
  } else {
    digitalWrite(LED_G1, HIGH);
    digitalWrite(LED_G2, HIGH);
    digitalWrite(LED_R1, LOW);
    digitalWrite(LED_R2, LOW);
  }

  // ======== PID SERVO CONTROL ========
  static double currentAngle = 92.20;
  static double previousError = 0.0;
  static double integral = 0.0;

  const double kp = 1.2;
  const double ki = 0.01;
  const double kd = 0.3;
  const double deadband = 0.5;

  double targetAngle = constrain(90.0 + pitch, 0, 180);
  double error = targetAngle - currentAngle;

  if (abs(error) > deadband) {
    integral += error;
    integral = constrain(integral, -50, 50);

    double derivative = error - previousError;
    previousError = error;

    double output = kp * error + ki * integral + kd * derivative;

    currentAngle += output;
    currentAngle = constrain(currentAngle, 0, 180);

    myServo.write(currentAngle);
  }

  // DEBUG
  Serial.print("Raw Pitch: ");
  Serial.print(rawPitch, 2);
  Serial.print(" | Pitch: ");
  Serial.print(pitch, 2);
  Serial.print(" | LED State: ");
  Serial.println(abs(pitchForLED) > tiltThreshold ? "RED" : "GREEN");
}