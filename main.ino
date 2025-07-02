#include <Wire.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID.h"
#include "BluetoothSerial.h"
#include <AccelStepper.h>

BluetoothSerial SerialBT;

AccelStepper motorLeft(AccelStepper::DRIVER, /*stepPin=*/9, /*dirPin=*/8);
AccelStepper motorRight(AccelStepper::DRIVER, /*stepPin=*/7, /*dirPin=*/6);

#define INTERRUPT_PIN       2
#define PPR                 1600
#define TICKS_PER_SECOND    50000  
#define MAX_ACCEL           200
#define ANGLE_Kp            1150.0
#define ANGLE_Kd            157.5
#define ANGLE_Ki            0.12
#define VELOCITY_Kp         0.1
#define VELOCITY_Kd         0.002
#define VELOCITY_Ki         0.0005

#define WARMUP_DELAY_US     (5000000UL)
#define ANGLE_SET_POINT     (2.0 * DEG_TO_RAD)
#define MAX_PACKET_SIZE     96

MPU6050        mpu;
volatile bool  mpuInterrupt = false;
Quaternion     q;
VectorFloat    gravity;
float          ypr[3];

float ref_velocity  = 0.0f;
float ref_steering  = 0.0f;
float steering      = 0.0f;
float accel         = 0.0;
float velocity      = 0.0;
bool  isBalancing   = false;
float angle         = 0.0;
float targetAngle   = ANGLE_SET_POINT;
String theString;

void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

void initMPU() {
  const int16_t accel_offset[3] = {-5508, -730, 666};
  const int16_t gyro_offset[3]  = {7, 18, -10};
  Wire.begin(); Wire.setClock(400000);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpu.setXGyroOffset(gyro_offset[0]);
  mpu.setYGyroOffset(gyro_offset[1]);
  mpu.setZGyroOffset(gyro_offset[2]);
  mpu.setXAccelOffset(accel_offset[0]);
  mpu.setYAccelOffset(accel_offset[1]);
  mpu.setZAccelOffset(accel_offset[2]);
  if (!mpu.testConnection()) { Serial.println("MPU6050 failed"); while(1); }
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  Serial.println("MPU6050 ready");
}

void moveSmoothDegrees(int target[4]) {
  bool busy = true;
  while (busy) {
    busy = false;
    for (int i = 0; i < 4; i++) {
      if (currentAngle[i] < target[i]) {
        currentAngle[i]++;
        busy = true;
      } else if (currentAngle[i] > target[i]) {
        currentAngle[i]--;
        busy = true;
      }
      uint16_t pulse = map(currentAngle[i], 0, 180, SERVO_MIN, SERVO_MAX);
      pwm.setPWM(i, 0, pulse);
    }
    if (busy) delay(stepDelay);
  }
}

bool mpuUpdate() {
  static uint8_t fifoBuffer[64];
  if (mpuInterrupt && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuInterrupt = false;
    return true;
  }
  return false;
}

void updateControl(unsigned long nowMicros) {
  static unsigned long prevMicros = micros();
  if (nowMicros < WARMUP_DELAY_US) return;
  if (nowMicros - prevMicros < 1000) return;
  if (!mpuUpdate()) return;

  float dt = (nowMicros - prevMicros) * 1e-6;
  angle = ypr[1];


  isBalancing = fabs(angle - targetAngle) < (PI/18);
  if (!isBalancing) {
    accel = velocity = 0;
    prevMicros = nowMicros;
    return;
  }

  static PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
  static PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);

  // compute new targetAngle from velocity PID
  targetAngle = -velocityPID.getControl(velocity, dt);
  anglePID.setTarget(targetAngle);
  accel = anglePID.getControl(angle, dt);
  accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);

  prevMicros = nowMicros;
}

void updateVelocity(float dt) {
  // integrate accel
  velocity += accel * dt;

  
  static const float alpha = 0.99;
  static float targetVel = 0, targetSteer = 0;
  targetVel = alpha*targetVel + (1-alpha)*ref_velocity;
  targetSteer = alpha*targetSteer + (1-alpha)*ref_steering;
  steering = targetSteer;
  // actual motor speeds in steps/sec
  float leftSpeed  = (velocity - steering) * (TICKS_PER_SECOND / PPR);
  float rightSpeed = (velocity + steering) * (TICKS_PER_SECOND / PPR);

  motorLeft.setSpeed(leftSpeed);
  motorRight.setSpeed(rightSpeed);
}

void splitSerialInputMove() {
  int c1 = theString.indexOf(',');
  int c2 = theString.indexOf(',', c1+1);
  ref_velocity = theString.substring(0, c1).toFloat();
  ref_steering = theString.substring(c1+1, c2).toFloat();
}

String createSerialOut() {
  return "x" + String(ref_velocity) + "," + String(ref_steering);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot");
  initMPU();
  // configure steppers
  motorLeft.setMaxSpeed(10000);
  motorRight.setMaxSpeed(10000);
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUpdateMicros) * 1e-6;
  updateControl(now);
  updateVelocity(dt);
  motorLeft.runSpeed();
  motorRight.runSpeed();
  lastUpdateMicros = now;

  // Bluetooth command handling
  if (SerialBT.available()) {
    theString = SerialBT.readStringUntil('\n');
    if (theString.startsWith("mov")) {
      splitSerialInputMove();
      SerialBT.println(createSerialOut());
    }
  }
}
