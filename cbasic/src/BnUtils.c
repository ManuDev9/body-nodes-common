/*
 * MIT License
 *
 * Copyright (c) 2025 Manuel Bottini
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS"; WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "BnUtils.h"
#include "math.h"

// Function to compute a rotation matrix from Euler angles (XYZ order). Roll,
// pitch and yaw are in radians
void BnUtils_blender_euler_to_rotation_matrix_rad(double roll, double pitch, double yaw, double (*const Ro)[3]) {
    // counter rh wise rotations
    double Rx[3][3] = {{1, 0, 0}, {0, cos(roll), -sin(roll)}, {0, sin(roll), cos(roll)}};

    double Ry[3][3] = {{cos(pitch), 0, sin(pitch)}, {0, 1, 0}, {-sin(pitch), 0, cos(pitch)}};

    double Rz[3][3] = {{cos(yaw), -sin(yaw), 0}, {sin(yaw), cos(yaw), 0}, {0, 0, 1}};

    double Rtmp[3][3];
    BnUtils_multiplyRotationMatrices(Ry, Rx, Rtmp);
    BnUtils_multiplyRotationMatrices(Rz, Rtmp, Ro);
}

void BnUtils_blender_euler_to_rotation_matrix_degree(double roll, double pitch, double yaw, double (*const Ro)[3]) {
    BnUtils_blender_euler_to_rotation_matrix_rad(BnUtils_toRadians(roll), BnUtils_toRadians(pitch),
                                                 BnUtils_toRadians(yaw), Ro);
}

BnQuaternion_t BnUtils_createQuanternion(BnAxisConfig_t const *const axis_config, double const *const values) {
    double valuesTmp[4] = {values[0], values[1], values[2], values[3]};
    BnAxisConfig_apply_double(axis_config, valuesTmp);
    return BnQuaternion_create_array(valuesTmp);
}

// all vals are double[4]
void BnUtils_transformSensorQuat(double const *const sensorQuatVals, double *const firstQuatVals,
                                 double const *const startingQuatVals, double const *const envQuatVals,
                                 int const *const pureAxisConfig, double *const valuesOut) {

    BnAxisConfig_t axisConfig;
    BnAxisConfig_config_purearray(&axisConfig, pureAxisConfig);

    BnQuaternion_t const sensorQuat = BnQuaternion_create_array(sensorQuatVals);
    BnQuaternion_t const startingQuat = BnQuaternion_create_array(startingQuatVals);
    BnQuaternion_t const envQuat = BnQuaternion_create_array(envQuatVals);

    BnQuaternion_t firstQuat = BnQuaternion_create_array(firstQuatVals);
    if (BnQuaternion_is_empty(&firstQuat) == 1) {
        firstQuat = BnQuaternion_inverse(&sensorQuat);
        BnQuaternion_to_list(&firstQuat, firstQuatVals);
    }

    BnQuaternion_t const rotationRealQuat = BnQuaternion_mul(&firstQuat, &sensorQuat);
    double valuesTmp[4];
    BnQuaternion_to_list(&rotationRealQuat, valuesTmp);
    BnQuaternion_t const rotationRealaxisQuat = BnUtils_createQuanternion(&axisConfig, valuesTmp);

    BnQuaternion_t objectNewQuat = BnQuaternion_inverse(&envQuat);
    objectNewQuat = BnQuaternion_mul(&objectNewQuat, &startingQuat);
    objectNewQuat = BnQuaternion_mul(&rotationRealaxisQuat, &objectNewQuat);
    objectNewQuat = BnQuaternion_mul(&envQuat, &objectNewQuat);
    BnQuaternion_to_list(&objectNewQuat, valuesOut);
}
