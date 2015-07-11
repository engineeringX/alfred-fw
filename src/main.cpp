#include "Arduino.h"
#include "RFduinoBLE.h"
#include "MPU6050.h"

#define TMP007_DEV_ADDR (0x40)
#define TMP007_REG_OBJTEMP (0x03)
#define TMP007_REG_DEVICE_ID (0x1F)

#define MPU6050_ACCEL_FS_2_LSB (16384.0f)
#define MPU6050_GYRO_FS_250_LSB (131.0f)

MPU6050 mpu;

void read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t* data) {
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

uint8_t read_byte(uint8_t dev_addr, uint8_t reg_addr) 
{
  uint8_t data;
  read_bytes(dev_addr, reg_addr, 1, &data);
  return data;
}

void write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t* data) 
{
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  for(uint8_t i = 0; i < length; i++) 
  {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

void write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) 
{
  write_bytes(dev_addr, reg_addr, 1, &data);
}

float tmp007_read_obj_temp() 
{
  int16_t regVal = read_byte(TMP007_DEV_ADDR, TMP007_REG_OBJTEMP);
  regVal = (regVal << 8) | read_byte(TMP007_DEV_ADDR, TMP007_REG_OBJTEMP);

  return (regVal >> 2) * 0.03125;
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


  mpu.initialize();
  mpu.setTempSensorEnabled(false);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
 
  float sample_rate = 1; // 1 Hz
  uint8_t dlpf_cfg = mpu.getDLPFMode();
  uint8_t smplrt_div = -1 + ((dlpf_cfg == 0 || dlpf_cfg == 7) ? 8000.0 : 1000.0) / sample_rate;
  mpu.setRate(smplrt_div);

  mpu.setIntDataReadyEnabled(true);

  //mpu.setXGyroFIFOEnabled(true);
  //mpu.setYGyroFIFOEnabled(true);
  //mpu.setZGyroFIFOEnabled(true);
  //mpu.setAccelFIFOEnabled(true);
  //mpu.setFIFOEnabled(true);
}

void loop() {
  //float temp = tmp007_read_obj_temp();

  //Serial.print("Temperature: ");
  //Serial.print(temp, DEC);
  //Serial.print("C");
  //Serial.print("\n\r");

  int16_t ax, ay, az, gx, gy, gz;

  if(!mpu.getIntDataReadyStatus()) return;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //uint8_t data[12];
  //mpu.getFIFOBytes(data, 12);

  //ax = ((int16_t)data[0] << 8) | data[1];
  //ay = ((int16_t)data[2] << 8) | data[3];
  //az = ((int16_t)data[4] << 8) | data[5];
  //
  //gx = ((int16_t)data[6] << 8) | data[7];
  //gy = ((int16_t)data[8] << 8) | data[9];
  //gz = ((int16_t)data[10] << 8) | data[11];

  Serial.print("{");
  Serial.print(ax/MPU6050_ACCEL_FS_2_LSB, DEC);
  Serial.print(", ");
  Serial.print(ay/MPU6050_ACCEL_FS_2_LSB, DEC);
  Serial.print(", ");
  Serial.print(az/MPU6050_ACCEL_FS_2_LSB, DEC);
  Serial.print("}");

  Serial.print("\t|\t");
  Serial.print("{");
  Serial.print(gx/MPU6050_GYRO_FS_250_LSB, DEC);
  Serial.print(", ");
  Serial.print(gy/MPU6050_GYRO_FS_250_LSB, DEC);
  Serial.print(", ");
  Serial.print(gz/MPU6050_GYRO_FS_250_LSB, DEC);
  Serial.println("}");
}

