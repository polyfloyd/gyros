#include "l3g4200d.h"
#include <SPI.h>

const int PIN_CS = 9;

float dpsPerDigit;

void l3g4200d_cmd(uint8_t rd, uint8_t addr, uint8_t *buf, int bufsize) {
    uint8_t ms = 1;
    const SPISettings spiSettings(500000, MSBFIRST, SPI_MODE3);
    pinMode(PIN_CS, HIGH);
    SPI.beginTransaction(spiSettings);
    SPI.transfer(rd<<7 | ms<<6 | addr);
    SPI.transfer(buf, bufsize);
    SPI.endTransaction();
    pinMode(PIN_CS, LOW);
}

void l3g4200d_setup() {
    int odr      = 0xa;
    int dpsScale = L3G4200D_DPS_2000;
    float dpsPerDigitTable[] = { .00875f, .0175f, .07f, };
    dpsPerDigit = dpsPerDigitTable[dpsScale];

    SPI.begin();
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    uint8_t whoami;
    do {
        l3g4200d_cmd(1, 0x0f, &whoami, 1);
    } while (whoami != 0b11010011);

    uint8_t reg1 = 0x0f | (odr << 4);
    l3g4200d_cmd(0, 0x20, &reg1, 1);
    uint8_t reg2 = 0x00;
    l3g4200d_cmd(0, 0x21, &reg2, 1);
    uint8_t reg3 = 0x00;
    l3g4200d_cmd(0, 0x22, &reg3, 1);
    uint8_t reg4 = dpsScale << 4;
    l3g4200d_cmd(0, 0x23, &reg4, 1);
    uint8_t reg5 = 0x00;
    l3g4200d_cmd(0, 0x24, &reg5, 1);
}

void l3g4200d_read_gyro(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    l3g4200d_cmd(1, 0x28, buf, 6);
    *x = buf[0] | buf[1]<<8;
    *y = buf[2] | buf[3]<<8;
    *z = buf[4] | buf[5]<<8;
}

void l3g4200d_calibrate(int n, float *threshX, float *threshY, float *threshZ) {
    float sumX = 0, sumY = 0, sumZ = 0;
    float squaredX = 0, squaredY = 0, squaredZ = 0;

    for (int i = 0; i < n; i++) {
        int16_t x, y, z;
        l3g4200d_read_gyro(&x, &y, &z);
        sumX += x;
        sumY += y;
        sumZ += z;
        squaredX += x * x;
        squaredY += y * y;
        squaredZ += z * z;
        delay(5);
    }

    float deltaX = sumX / n;
    float deltaY = sumY / n;
    float deltaZ = sumZ / n;
    *threshX = sqrt(squaredX / n) * 2;
    *threshY = sqrt(squaredY / n) * 2;
    *threshZ = sqrt(squaredZ / n) * 2;
}
