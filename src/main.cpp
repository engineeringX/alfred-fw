#include "Arduino.h"
#include "RFduinoBLE.h"
#include "MPU6050.h"
#include <deque>

#define TMP007_DEV_ADDR (0x40)
#define TMP007_REG_OBJTEMP (0x03)
#define TMP007_REG_DEVICE_ID (0x1F)

#define TMP007_LSB (0.03125)

#define MPU6050_ACCEL_FS_2_LSB (16384.0f)
#define MPU6050_GYRO_FS_250_LSB (131.0f)

#define MPU6050_DATA_IRQ (2)

#define HALFWORDS_PER_SAMPLE (8)
#define BYTES_PER_SAMPLE (12)
#define BUF_SIZE (1022)

#define FILTER_LENGTH (50)
#define SEC_FILTER_LENGTH (70)
#define FALL_THRESH_LOW (100)
#define FALL_THRESH_HIGH (550)
#define PUSH_NOTIF_LIMIT (20)

MPU6050 mpu;

//int16_t buf[BUF_SIZE];
//size_t count;
uint16_t packetCount = 0;
uint16_t dataPoint = 0;
std::deque<int16_t> linesFIFO;
std::deque<int32_t> sumData;
int16_t weights[] = {-7, -19, -26, -22, -1, 32, 65, 73, 40, -34, -117, -164, -20, 134,
                     253, 263, 141, -72, -276, -365, -284, -61, 202, 376, 376, 202, -61,
                     -284, -365, -276, -72, 141, 263, 253, 134, -20, -134, -164, -117, -34, 40,
                     73, 65, 32, -1, -22, -26, -19, -7};

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

void ble_send(uint8_t* data, size_t size, uint32_t ms)
{
    // this is the data we want to appear in the advertisement
    // (if the deviceName and advertisementData are too long to fix into the 31 byte
    // ble advertisement packet, then the advertisementData is truncated first down to
    // a single byte, then it will truncate the deviceName)
    uint8_t bytes = 0;
    uint8_t buf[31];

    // Device name: 'a'
    buf[bytes++] = 0x02;
    buf[bytes++] = 0x09;
    buf[bytes++] = 0x41;

    // flags: general discovery mode | br edr not supported 
    buf[bytes++] = 0x02;
    buf[bytes++] = 0x01;
    buf[bytes++] = 0x06;

    // TX power level: +4 dbm
    buf[bytes++] = 0x02;
    buf[bytes++] = 0x0a;
    buf[bytes++] = 0x04;

    // 16-bit service class UUID's
    //buf[bytes++] = 0x03;
    //buf[bytes++] = 0x03;
    //buf[bytes++] = 0x20;
    //buf[bytes++] = 0x22;

    // Sensor data
    buf[bytes++] = size+1;
    buf[bytes++] = 0xff;
    memcpy(buf+bytes, data, size);
    RFduinoBLE_advdata = buf;
    RFduinoBLE_advdata_len = bytes+size;

    // start the BLE stack
    RFduinoBLE.begin();

    // advertise for ms milliseconds
    RFduino_ULPDelay(ms);

    // stop the BLE stack
    RFduinoBLE.end();
}

void filterInit() {

}

int main() {
    init();

    filterInit();

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

    mpu.initialize();
    mpu.setTempSensorEnabled(false);
    mpu.setDLPFMode(MPU6050_DLPF_BW_256);
    mpu6050_set_sample_rate(20); // 50 Hz sample rate

    mpu.setXGyroFIFOEnabled(true);
    mpu.setYGyroFIFOEnabled(true);
    mpu.setZGyroFIFOEnabled(true);
    mpu.setAccelFIFOEnabled(true);
    mpu.setFIFOEnabled(true);

    mpu.setIntDataReadyEnabled(true);

    //pinMode(MPU6050_DATA_IRQ, INPUT);
    //RFduino_pinWakeCallback(MPU6050_DATA_IRQ, HIGH, mpu6050_data_ready);
    //mpu.getIntStatus();
}

void motionFilter(int16_t* buf) {
    dataPoint += 1;
    int32_t outputSignal = 0;

    // FIXME: We need at least 50 readings in linesFIFO 
    if (linesFIFO.size() >= FILTER_LENGTH) {
        for(uint8_t sample=0; sample<FILTER_LENGTH; ++sample) {
            outputSignal = outputSignal + weights[sample] * linesFIFO[sample];
        }
        Serial.print(dataPoint);
        Serial.print(":");
        Serial.print(outputSignal);
        Serial.println();

        sumData.push_back(outputSignal);

        if (sumData.size() >= SEC_FILTER_LENGTH) {
            int16_t total_sum_data = 0;
            for (uint8_t elem=0; elem < sumData.size(); elem++) {
                total_sum_data += elem;
            }
            if (total_sum_data >= FALL_THRESH_HIGH || total_sum_data <= FALL_THRESH_LOW) {
                Serial.println("Fall detected");
                Serial.print("Total Sum Data: ");
                Serial.print(total_sum_data, DEC);
                packetCount++;
            }
            sumData.pop_front();
        }

        linesFIFO.pop_front();
        if (sizeof(buf) >= HALFWORDS_PER_SAMPLE+2) {
            linesFIFO.push_back(buf[2]);
        }
    } else {    // We don't have enough readings yet, so fill up the buffer.
        // Check if we have enough data in the FIFO
        if (sizeof(buf) >= HALFWORDS_PER_SAMPLE+2)
            linesFIFO.push_back(buf[2]);
    }
}

void printSerial(int16_t* buf) {
    Serial.print(buf[0], DEC);
    Serial.print(",");
    Serial.print(buf[1], DEC);
    Serial.print(",");
    Serial.print(buf[2], DEC);
    Serial.print(",");
    Serial.print(buf[3], DEC);
    Serial.print(",");
    Serial.print(buf[4], DEC);
    Serial.print(",");
    Serial.print(buf[5], DEC);
    Serial.print(",");
    Serial.print(buf[6], DEC);
    Serial.print(",");
    Serial.print(buf[7], DEC);
    Serial.println();
}

void loop() {

    int16_t buf[HALFWORDS_PER_SAMPLE+2];

    if(!mpu.getIntDataReadyStatus()) return;
    mpu.getMotion6(buf, buf+1, buf+2, buf+3, buf+4, buf+5);
    *(buf+6) = tmp007_read_obj_temp();
    *(buf+7) = analogRead(2);

    //printSerial(buf);
    motionFilter(buf);

    if (!(packetCount % PUSH_NOTIF_LIMIT)) {
        // TODO: Decide on a format of what it means to have a buffer value for fall.
        memset(buf, 0, sizeof(buf));
        ble_send((uint8_t*)buf, HALFWORDS_PER_SAMPLE << 1, 20);
    }
}

