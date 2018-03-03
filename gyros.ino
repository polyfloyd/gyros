#include "linmath.h"
#include "l3g4200d.h"

vec3 threshold;
quat orientation;

void setup() {
    quat_identity(orientation);
    Serial.begin(230400);

    Serial.println("# waiting for gyrosensor...");
    l3g4200d_setup();
    Serial.println("# gyrosensor online");
    l3g4200d_calibrate(100, &threshold[0], &threshold[1], &threshold[2]);
}

void loop() {
    int16_t x, y, z;
    l3g4200d_read_gyro(&x, &y, &z);

    if (abs(x) < threshold[0]) x = 0;
    if (abs(y) < threshold[1]) y = 0;
    if (abs(z) < threshold[2]) z = 0;

    if (x != 0 || y != 0 || z != 0) {
        quat tmp;
        quat qx;
        const static vec3 axisX = {1, 0, 0};
        quat_rotate(qx, float(x) / 32000.0, axisX);
        memcpy(tmp, orientation, sizeof(quat));
        quat_mul(orientation, qx, tmp);

        quat qy;
        const static vec3 axisY = {0, 1, 0};
        quat_rotate(qy, float(y) / 32000.0, axisY);
        memcpy(tmp, orientation, sizeof(quat));
        quat_mul(orientation, qy, tmp);

        quat qz;
        const static vec3 axisZ = {0, 0, 1};
        quat_rotate(qz, float(z) / 32000.0, axisZ);
        memcpy(tmp, orientation, sizeof(quat));
        quat_mul(orientation, qz, tmp);

        mat4x4 matrix;
        mat4x4_from_quat(matrix, orientation);
        Serial.print("mat4");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                Serial.print(" ");
                Serial.print(matrix[i][j]);
            }
        }
        Serial.println();
    }

    delay(2);
}
