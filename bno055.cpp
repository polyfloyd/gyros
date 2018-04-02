#include <Arduino.h>
#include <Wire.h>
#include "bno055.h"

#define BNO055_ADDR 0x29
#define BNO055_ID 0xa0

#define BNO055_REG_ID 0x00
#define BNO055_REG_PAGE_ID 0x07
#define BNO055_REG_QUA_DATA_W_LSB 0x20
#define BNO055_REG_ST_RESULT 0x36
#define BNO055_REG_SYS_ERR 0x3a
#define BNO055_REG_OPR_MODE 0x3d
#define BNO055_REG_PWR_MODE 0x3e
#define BNO055_REG_SYS_TRIGGER 0x3f
#define BNO055_REG_AXIS_MAP_CONFIG 0x41
#define BNO055_REG_AXIS_MAP_SIGN 0x42

#define BNO055_OPMODE_CONFIG 0x00
#define BNO055_OPMODE_NDOF 0x0c
#define BNO055_OPMODE_IMU 0x08
#define BNO055_OPMODE_M4G 0x0a
#define BNO055_PWRMODE_NORMAL 0

void bno055_read(uint8_t reg, uint8_t *buf, size_t bufLen);
uint8_t bno055_read8(uint8_t reg);
void bno055_write(uint8_t reg, uint8_t *buf, size_t bufLen);
void bno055_write8(uint8_t reg, uint8_t val);
void bno055_wait_powerup();

int bno055_setup() {
    Wire.begin();

    Serial.println("# waiting for gyrosensor...");
    bno055_wait_powerup();
    Serial.println("# gyrosensor online");

    // Switch to config mode, in case of an in proper power cycle.
    bno055_write8(BNO055_REG_OPR_MODE, BNO055_OPMODE_CONFIG);
    delay(30);
    // Reset all values to default and run a self test.
    bno055_write8(BNO055_REG_SYS_TRIGGER, 0x61); // RST_INT, RST_SYS and self test
    bno055_wait_powerup();
    delay(50);

    // Analyze self test results.
    uint8_t selftest_result = ~bno055_read8(BNO055_REG_ST_RESULT);
    if (selftest_result & 0x0f) {
        if (selftest_result & 1) {
            Serial.println("# Accelerometer self test failed");
        }
        if (selftest_result & 2) {
            Serial.println("# Magnetometer self test failed");
        }
        if (selftest_result & 4) {
            Serial.println("# Gyroscope self test failed");
        }
        if (selftest_result & 8) {
            Serial.println("# MCU self test failed");
        }
        return selftest_result;
    }
    // Check for general system errors.
    uint8_t err = bno055_read8(BNO055_REG_SYS_ERR);
    switch (err) {
    case 0:
        Serial.println("# No error");
        break;
    case 1:
        Serial.println("# Peripheral initialization error");
        break;
    case 2:
        Serial.println("# System initialization error");
        break;
    case 3:
        Serial.println("# Self test result failed");
        break;
    case 4:
        Serial.println("# Register map value out of range");
        break;
    case 5:
        Serial.println("# Register map address out of range");
        break;
    case 6:
        Serial.println("# Register map write error");
        break;
    case 7:
        Serial.println("# BNO low power mode not available for selected operat ion mode");
        break;
    case 8:
        Serial.println("# Accelerometer power mode not available");
        break;
    case 9:
        Serial.println("# Fusion algorithm configuration error");
        break;
    case 10:
        Serial.println("# Sensor configuration error");
        break;
    default:
        Serial.print("# Unknown error: ");
        Serial.println(err);
    }
    if (err) {
        return err;
    }

    bno055_write8(BNO055_REG_PWR_MODE, BNO055_PWRMODE_NORMAL);
    bno055_write8(BNO055_REG_PAGE_ID, 0);
    bno055_write8(BNO055_REG_SYS_TRIGGER, 0);
    delay(10);

    // Switch to a mode to start reading sensor data.
    bno055_write8(BNO055_REG_OPR_MODE, BNO055_OPMODE_NDOF);
    delay(30);

    return 0;
}

void bno055_wait_powerup() {
    while (bno055_read8(BNO055_REG_ID) != BNO055_ID) {
        delay(5);
    }
}

void bno055_read_abs_orientation(quat out) {
    uint8_t buf[8];
    bno055_read(BNO055_REG_QUA_DATA_W_LSB, buf, 8);
    int16_t w = ((int16_t)buf[1]<<8) | (int16_t)buf[0];
    int16_t x = ((int16_t)buf[3]<<8) | (int16_t)buf[2];
    int16_t y = ((int16_t)buf[5]<<8) | (int16_t)buf[4];
    int16_t z = ((int16_t)buf[7]<<8) | (int16_t)buf[6];

    const float scale = 1.0 / (float)(1<<14);
    out[3] = (float)w * scale;
    out[0] = (float)x * scale;
    out[1] = (float)y * scale;
    out[2] = (float)z * scale;
}

void bno055_read(uint8_t reg, uint8_t *buf, size_t bufLen) {
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(BNO055_ADDR, bufLen);
    for (uint8_t *p = buf; p < buf + bufLen; p++) {
        *p = Wire.read();
    }
}

uint8_t bno055_read8(uint8_t reg) {
    uint8_t val;
    bno055_read(reg, &val, 1);
    return val;
}

void bno055_write(uint8_t reg, uint8_t *buf, size_t bufLen) {
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(reg);
    for (uint8_t *p = buf; p < buf + bufLen; p++) {
        Wire.write(*p);
    }
    Wire.endTransmission();
}

void bno055_write8(uint8_t reg, uint8_t val) {
    bno055_write(reg, &val, 1);
}
