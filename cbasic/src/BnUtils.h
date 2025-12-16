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

#ifndef BN_UTILS_H
#define BN_UTILS_H

#include "BnAxisConfig.h"
#include "BnConstants.h"
#include "BnQuaternion.h"

//////////////// MOTION TRACKING AND IK

static inline double BnUtils_toRadians(double const degrees) { return degrees * (BN_M_PI / 180.0); }

// Function to multiply two rotation matrices
// double const R1[3][3], double const R2[3][3], double Ro[3][3]
static inline void BnUtils_multiplyRotationMatrices(double const (*const R1)[3], double const (*const R2)[3],
                                                    double (*const Ro)[3]) {

    Ro[0][0] = R1[0][0] * R2[0][0] + R1[0][1] * R2[1][0] + R1[0][2] * R2[2][0];
    Ro[0][1] = R1[0][0] * R2[0][1] + R1[0][1] * R2[1][1] + R1[0][2] * R2[2][1];
    Ro[0][2] = R1[0][0] * R2[0][2] + R1[0][1] * R2[1][2] + R1[0][2] * R2[2][2];

    Ro[1][0] = R1[1][0] * R2[0][0] + R1[1][1] * R2[1][0] + R1[1][2] * R2[2][0];
    Ro[1][1] = R1[1][0] * R2[0][1] + R1[1][1] * R2[1][1] + R1[1][2] * R2[2][1];
    Ro[1][2] = R1[1][0] * R2[0][2] + R1[1][1] * R2[1][2] + R1[1][2] * R2[2][2];

    Ro[2][0] = R1[2][0] * R2[0][0] + R1[2][1] * R2[1][0] + R1[2][2] * R2[2][0];
    Ro[2][1] = R1[2][0] * R2[0][1] + R1[2][1] * R2[1][1] + R1[2][2] * R2[2][1];
    Ro[2][2] = R1[2][0] * R2[0][2] + R1[2][1] * R2[1][2] + R1[2][2] * R2[2][2];
}

// Function to multiply a rotation matrix and a vector
// double const R1[3][3], double const V1[3], double Vo[3]
static inline void BnUtils_multiplyRotationMatrixWithVector(double const (*const R1)[3], double const *const V1,
                                                            double *const Vo) {
    Vo[0] = R1[0][0] * V1[0] + R1[0][1] * V1[1] + R1[0][2] * V1[2];
    Vo[1] = R1[1][0] * V1[0] + R1[1][1] * V1[1] + R1[1][2] * V1[2];
    Vo[2] = R1[2][0] * V1[0] + R1[2][1] * V1[1] + R1[2][2] * V1[2];
}

// Function to compute a rotation matrix from Euler angles (XYZ order). Roll,
// pitch and yaw are in radians double Ro[3][3]
void BnUtils_blender_euler_to_rotation_matrix_rad(double roll, double pitch, double yaw, double (*const Ro)[3]);

// Function to compute a rotation matrix from Euler angles (XYZ order). Roll,
// pitch and yaw are in degrees double Ro[3][3]
void BnUtils_blender_euler_to_rotation_matrix_degree(double roll, double pitch, double yaw, double (*const Ro)[3]);

// double const values[4]
BnQuaternion_t BnUtils_createQuanternion(BnAxisConfig_t const *const axis_config, double const *const values);

/** This trasformation ignores what is the local axis of the object once
 * rotated, we don't trust those axis system which might be compromised
 * sensorQuatVals               - is the raw quaternion (vector of 4 values)
 * from the sensor firstQuatVals                - [inout] is the first
 * quaternion that has been registered from the sensor, this will create a 0
 * angle from where the sensor started sensing. Please fill it with 0,0,0,0 to
 * set it as initially empty startingQuatVals             - is the starting
 * quaternion of the object we want to rotated with the sensor envQuatVals - is
 * the environment quaternion, basically indicates somehow where the x axis
 * points pureAxisConfig               - axis configuration that will transfor
 * the sensor values into values for the virtual world. Composed of 8 int array
 * values that I can use to create a BnAxisConfig
 */

//   double sensorQuatVals[4],
//    double firstQuatVals[4],
//    double startingQuatVals[4],
//    double envQuatVals[4],
//    int pureAxisConfig[4],
//    double valuesOut[4] );
void BnUtils_transformSensorQuat(double const *const sensorQuatVals, double *const firstQuatVals,
                                 double const *const startingQuatVals, double const *const envQuatVals,
                                 int const *const pureAxisConfig, double *const valuesOut);

//////////////// DATATYPES, DATAKEYS and COMMUNICATION

// char key[8]
// char bodypart[4]
static inline uint8_t BnUtils_doesKeyContainBodypart(char const *const key, char const *const bodypart) {
    return (key[0] == bodypart[0] && key[1] == bodypart[1] && key[2] == bodypart[2] && key[3] == bodypart[3]);
}

static inline uint8_t BnUtils_doesKeyContainPlayer(char const *const key, char const *const player) {
    return (key[4] == player[0] && key[5] == player[1] && key[6] == player[2] && key[7] == player[3]);
}

#endif // BN_UTILS_H
