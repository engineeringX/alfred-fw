#include "Arduino.h"
#include "RFduinoBLE.h"
#include "MPU6050.h"

#define TMP007_DEV_ADDR (0x40)
#define TMP007_REG_OBJTEMP (0x03)
#define TMP007_REG_DEVICE_ID (0x1F)

#define TMP007_LSB (0.03125)

#define MPU6050_ACCEL_FS_2_LSB (16384.0f)
#define MPU6050_GYRO_FS_250_LSB (131.0f)

#define MPU6050_DATA_IRQ (2)

#define HALFWORDS_PER_SAMPLE (6)
#define BYTES_PER_SAMPLE (12)
#define BUF_SIZE (1022)

MPU6050 mpu;

//int16_t buf[BUF_SIZE];
//size_t count;

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

int16_t tmp007_read_obj_temp() 
{
  int16_t regVal = read_byte(TMP007_DEV_ADDR, TMP007_REG_OBJTEMP);
  regVal = (regVal << 8) | read_byte(TMP007_DEV_ADDR, TMP007_REG_OBJTEMP);

  return (regVal >> 2);
}

void mpu6050_set_sample_rate(uint16_t sample_rate)
{
  uint8_t dlpf_cfg = mpu.getDLPFMode();
  uint8_t smplrt_div = -1 + ((dlpf_cfg == 0 || dlpf_cfg == 7) ? 8000.0 : 1000.0) / sample_rate;
  mpu.setRate(smplrt_div);
}

void advertise(uint8_t* data, uint32_t ms)
{
  // this is the data we want to appear in the advertisement
  // (if the deviceName and advertisementData are too long to fix into the 31 byte
  // ble advertisement packet, then the advertisementData is truncated first down to
  // a single byte, then it will truncate the deviceName)
  uint8_t buf[31];
  buf[0] = 0x02;
  buf[1] = 0x09;
  buf[2] = 0x41;
  buf[3] = 0x02;
  buf[4] = 0x01;
  buf[5] = 0x06;
  buf[6] = 0x02;
  buf[7] = 0x0a;
  buf[8] = 0x04;
  buf[9] = 0x03;
  buf[10] = 0x03;
  buf[11] = 0x20;
  buf[12] = 0x22;
  buf[13] = 0x0d;
  buf[14] = 0xff;
  memcpy(buf+15, data, 12);
  RFduinoBLE_advdata = buf;
  RFduinoBLE_advdata_len = 27;

  // start the BLE stack
  RFduinoBLE.begin();
  
  // advertise for ms milliseconds
  RFduino_ULPDelay(ms);
  
  // stop the BLE stack
  RFduinoBLE.end();
}

int main() {
  init();

  setup();

  for(;;) {
    loop();
  }

  return 0;
}

//int mpu6050_data_ready(long unsigned int unused)
//{
//  //if(!mpu.getIntDataReadyStatus()) return unused;
//  mpu.getIntDataReadyStatus();
//  Serial.println("WORKING");
//  if(count < BUF_SIZE)
//  {
//    mpu.getMotion6(buf+count, buf+count+1, buf+count+2, buf+count+3, buf+count+4, buf+count+5);
//    count += 6;
//  }
//
//  return unused;
//}

void setup() {
  //RFduino_systemReset();
  Serial.begin(9600);
  Wire.begin();

  //count = 0;

  mpu.initialize();
  mpu.setTempSensorEnabled(false);
  mpu.setDLPFMode(MPU6050_DLPF_BW_256);
  mpu6050_set_sample_rate(100); // 1 Hz sample rate
  
  mpu.setIntDataReadyEnabled(true);

  //pinMode(MPU6050_DATA_IRQ, INPUT);
  //RFduino_pinWakeCallback(MPU6050_DATA_IRQ, HIGH, mpu6050_data_ready);
  //mpu.getIntStatus();

  //mpu.setXGyroFIFOEnabled(true);
  //mpu.setYGyroFIFOEnabled(true);
  //mpu.setZGyroFIFOEnabled(true);
  //mpu.setAccelFIFOEnabled(true);
  //mpu.setFIFOEnabled(true);
}

void loop() {
  RFduino_ULPDelay(1000);

  //float temp = tmp007_read_obj_temp();

  //Serial.print("Temperature: ");
  //Serial.print(temp, DEC);
  //Serial.print("C");
  //Serial.print("\n\r");

  int16_t buf[HALFWORDS_PER_SAMPLE+2];

  if(!mpu.getIntDataReadyStatus()) return;
  mpu.getMotion6(buf, buf+1, buf+2, buf+3, buf+4, buf+5);
  
  advertise((uint8_t*)buf, 5000);

  //*(buf+6) = tmp007_read_obj_temp();
  //Serial.println(buf[6]*TMP007_LSB, DEC);

  //Serial.write((uint8_t*)buf, BYTES_PER_SAMPLE); 
  
  //uint8_t data[12];
  //mpu.getFIFOBytes(data, 12);

  //ax = ((int16_t)data[0] << 8) | data[1];
  //ay = ((int16_t)data[2] << 8) | data[3];
  //az = ((int16_t)data[4] << 8) | data[5];
  //
  //gx = ((int16_t)data[6] << 8) | data[7];
  //gy = ((int16_t)data[8] << 8) | data[9];
  //gz = ((int16_t)data[10] << 8) | data[11];
  
  //if(count < 6) return;

  Serial.print(buf[0], DEC);
  Serial.print(", ");
  Serial.print(buf[1], DEC);
  Serial.print(", ");
  Serial.print(buf[2], DEC);
  Serial.print(",");
  Serial.print(buf[3], DEC);
  Serial.print(", ");
  Serial.print(buf[4], DEC);
  Serial.print(", ");
  Serial.print(buf[5], DEC);
  Serial.println();
  
  //count -= 6;
}

