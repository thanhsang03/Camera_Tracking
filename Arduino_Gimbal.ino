#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#include <Servo.h>

// MPU6050 variables
bool dmpReady = false;       // Indicates if DMP initialization was successful
uint8_t mpuIntStatus;        // Holds actual interrupt status byte from MPU6050
uint8_t devStatus;           // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;         // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;          // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];      // FIFO storage buffer

// Orientation/motion variables
Quaternion quaternion;
VectorFloat gravityVector;
float yawPitchRoll[3];

volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

#define INTERRUPT_PIN 2

MPU6050 mpu;

Servo yawServo;
Servo pitchServo;
Servo rollServo;

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // Set your own offset values for stable output
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(80);
  mpu.setZAccelOffset(1450);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  yawServo.attach(6);
  pitchServo.attach(3);
  rollServo.attach(5);
}

void loop() {
  if (!dmpReady) return;
  
  readMPU6050();

  // Map sensor values to servo positions
  int yawAngle = map(yawPitchRoll[0], -90, 90, 0, 180);
  int pitchAngle = map(yawPitchRoll[1], -90, 90, 180, 0);
  int rollAngle = map(yawPitchRoll[2], -90, 90, 0, 180);

  yawServo.write(constrain(yawAngle, 0, 180));
  pitchServo.write(constrain(pitchAngle, 0, 180));
  rollServo.write(constrain(rollAngle, 0, 180));

  /*Serial.print("yaw : ");
  Serial.print(yawAngle);
  Serial.print(", pitch : ");
  Serial.print(pitchAngle);
  Serial.print(", roll : ");
  Serial.println(rollAngle);*/
}

float initialYaw;
int count = 0;

void readMPU6050() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
    mpu.dmpGetGravity(&gravityVector, &quaternion);
    mpu.dmpGetYawPitchRoll(yawPitchRoll, &quaternion, &gravityVector);

    yawPitchRoll[0] = yawPitchRoll[0] * 180 / M_PI;
    yawPitchRoll[1] = yawPitchRoll[1] * 180 / M_PI;
    yawPitchRoll[2] = yawPitchRoll[2] * 180 / M_PI;

    if (count <= 300) {
      count++;
      initialYaw = yawPitchRoll[0];
      readMPU6050();
      return;
    }
    yawPitchRoll[0] -= initialYaw;
  }
}
