#include "linmath.h"
#include "bno055.h"

vec3 threshold;
quat orientation;

static mat4x4 axis_mapping = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};

void setup() {
    quat_identity(orientation);
    Serial.begin(230400);

    if (bno055_setup()) {
        Serial.println("# Error initializing BNO055");
        while (1);
    }
}

void loop() {
    delay(5);

    static quat prev_orientation;
    quat orientation;
    bno055_read_abs_orientation(orientation);
    if (!memcmp(prev_orientation, orientation, sizeof(quat))) {
        return;
    }
    memcpy(prev_orientation, orientation, sizeof(quat));

    mat4x4 input, output;
    mat4x4_from_quat(input, orientation);
    mat4x4_mul(output, input, axis_mapping);

    Serial.print("mat4");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Serial.print(" ");
            Serial.print(output[i][j]);
        }
    }
    Serial.println();
}
