#include "Arduino.h"
#include "Wire.h"
#include "RFduinoBLE.h"

#include "tuple.hpp"

#define TMP007_DEV_ADDR (0x40)
#define TMP007_REG_OBJTEMP (0x03)
#define TMP007_REG_DEVICE_ID (0x1F)

#define MPU6050_DEV_ADDR (0x68)
#define MPU6050_REG_ACCEL_CONFIG (0x1C)
#define MPU6050_REG_GYRO_CONFIG (0x1B)
#define MPU6050_REG_ACCEL_X_HI (0x3B) 
#define MPU6050_REG_ACCEL_X_LO (0x3C)
#define MPU6050_REG_ACCEL_Y_HI (0x3D)
#define MPU6050_REG_ACCEL_Y_LO (0x3E)
#define MPU6050_REG_ACCEL_Z_HI (0x3F)
#define MPU6050_REG_ACCEL_Z_LO (0x40)
#define MPU6050_REG_GYRO_X_HI (0x43) 
#define MPU6050_REG_GYRO_X_LO (0x44)
#define MPU6050_REG_GYRO_Y_HI (0x45)
#define MPU6050_REG_GYRO_Y_LO (0x46)
#define MPU6050_REG_GYRO_Z_HI (0x47)
#define MPU6050_REG_GYRO_Z_LO (0x48)
#define MPU6050_REG_PWRMGMT_01 (0x6B)
#define MPU6050_REG_WHO_AM_I (0x75)

void read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, int8_t* data) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.beginTransmission(dev_addr);
  Wire.requestFrom(dev_addr, length);
  for(uint8_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }
  Wire.endTransmission();
}

int8_t read_byte(uint8_t dev_addr, uint8_t reg_addr) {
  int8_t data;
  read_bytes(dev_addr, reg_addr, 1, &data);
  return data;
}

void write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, int8_t* data) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  for(uint8_t i = 0; i < length; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

void write_byte(uint8_t dev_addr, uint8_t reg_addr, int8_t data) {
  write_bytes(dev_addr, reg_addr, 1, &data);
}

float tmp007_read_obj_temp() {
  int16_t regVal = read_byte(TMP007_DEV_ADDR, TMP007_REG_OBJTEMP);
  regVal = (regVal << 8) | read_byte(TMP007_DEV_ADDR, TMP007_REG_OBJTEMP);

  return (regVal >> 2) * 0.03125;
}

void mpu6050_set_sleep_enable(bool sleep)
{
  uint8_t data = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_PWRMGMT_01);
  data = ((uint8_t)sleep << 6) | (data & ~(1 << 6));
  write_byte(MPU6050_DEV_ADDR, MPU6050_REG_PWRMGMT_01, data);
}

void mpu6050_init_config()
{
  uint8_t pwrmgmt = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_PWRMGMT_01);
  write_byte(MPU6050_DEV_ADDR, MPU6050_REG_PWRMGMT_01, (pwrmgmt & ~(0x7)) | 0x01);

  uint8_t config = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_CONFIG);
  write_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_CONFIG, config & ~(0x3 << 3));

  config = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_CONFIG);
  write_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_CONFIG, config & ~(0x3 << 3));
}

Triple<int16_t, int16_t, int16_t> mpu6050_read_accel()
{
  int16_t x_accel = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_X_HI);
  x_accel = (x_accel << 8) | read_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_X_LO);

  int16_t y_accel = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_Y_HI);
  y_accel = (y_accel << 8) | read_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_Y_LO);

  int16_t z_accel = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_Z_HI);
  z_accel = (z_accel << 8) | read_byte(MPU6050_DEV_ADDR, MPU6050_REG_ACCEL_Z_LO);

  return Triple<int16_t, int16_t, int16_t>(x_accel, y_accel, z_accel);
}

Triple<int16_t, int16_t, int16_t> mpu6050_read_gyro()
{
  int16_t x_gyro = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_X_HI);
  x_gyro = (x_gyro << 8) | read_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_X_LO);

  int16_t y_gyro = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_Y_HI);
  y_gyro = (y_gyro << 8) | read_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_Y_LO);

  int16_t z_gyro = read_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_Z_HI);
  z_gyro = (z_gyro << 8) | read_byte(MPU6050_DEV_ADDR, MPU6050_REG_GYRO_Z_LO);

  return Triple<int16_t, int16_t, int16_t>(x_gyro, y_gyro, z_gyro);
}

int main() {
  init();

  setup();

  for(;;) {
    loop();
  }

  return 0;
}

void setup() {
  //RFduino_systemReset();
  Serial.begin(9600);
  Wire.begin();

  mpu6050_init_config();
  mpu6050_set_sleep_enable(false);
}

void loop() {
  delay(100);
 
  //float temp = tmp007_read_obj_temp();

  //Serial.print("Temperature: ");
  //Serial.print(temp, DEC);
  //Serial.print("C");
  //Serial.print("\n\r");
  
  Triple<int16_t, int16_t, int16_t> accel = mpu6050_read_accel();
  Serial.print("{");
  Serial.print(accel.first(), DEC);
  Serial.print(", ");
  Serial.print(accel.second(), DEC);
  Serial.print(", ");
  Serial.print(accel.third(), DEC);
  Serial.print("}, ");

  Triple<int16_t, int16_t, int16_t> gyro = mpu6050_read_gyro();
  Serial.print("{");
  Serial.print(gyro.first(), DEC);
  Serial.print(", ");
  Serial.print(gyro.second(), DEC);
  Serial.print(", ");
  Serial.print(gyro.third(), DEC);
  Serial.println("}");

}

