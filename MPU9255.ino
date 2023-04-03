#include <Wire.h>

const int MPU9255_ADDR = 0x68; // I2C address of MPU9255

// Register addresses for MPU9255
const int ACCEL_XOUT_H = 0x3B;
const int GYRO_XOUT_H = 0x43;
const int MAG_XOUT_H = 0x03;

int16_t accel_x, accel_y, accel_z; // Raw accelerometer data
int16_t gyro_x, gyro_y, gyro_z; // Raw gyroscope data
int16_t mag_x, mag_y, mag_z; // Raw magnetometer data

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU9255_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // Wake up MPU9255
  Wire.endTransmission(true);
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU9255_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9255_ADDR, 6, true);
  accel_x = Wire.read() << 8 | Wire.read();
  accel_y = Wire.read() << 8 | Wire.read();
  accel_z = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(MPU9255_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9255_ADDR, 6, true);
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  // Read magnetometer data
  Wire.beginTransmission(MPU9255_ADDR);
  Wire.write(MAG_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9255_ADDR, 6, true);
  mag_x = Wire.read() << 8 | Wire.read();
  mag_y = Wire.read() << 8 | Wire.read();
  mag_z = Wire.read() << 8 | Wire.read();

  // Print data to serial monitor
  Serial.print("Accelerometer: ");
  Serial.print(accel_x);
  Serial.print(", ");
  Serial.print(accel_y);
  Serial.print(", ");
  Serial.println(accel_z);
  Serial.print("Gyroscope: ");
  Serial.print(gyro_x);
  Serial.print(", ");
  Serial.print(gyro_y);
  Serial.print(", ");
  Serial.println(gyro_z);
  Serial.print("Magnetometer: ");
  Serial.print(mag_x);
  Serial.print(", ");
  Serial.print(mag_y);
  Serial.print(", ");
  Serial.println(mag_z);

  delay(500); // Delay between readings
}
