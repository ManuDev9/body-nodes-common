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

 * The above copyright notice and this permission notice shall be included in all
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
#include "cmath"

namespace bodynodesdev {

namespace common {

// Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in radians
void BnUtils::blender_euler_to_rotation_matrix_rad(double roll, double pitch, double yaw, double (* const Ro)[3]) {
    // counter rh wise rotations
    double Rx[3][3] = 
    {
        { 1, 0, 0 },
        { 0, std::cos(roll), -std::sin(roll) },
        { 0, std::sin(roll), std::cos(roll) }
    };

    double Ry[3][3] =
    {
        { std::cos(pitch), 0, std::sin(pitch) },
        { 0, 1, 0 },
        { -std::sin(pitch), 0, std::cos(pitch) }
    };

    double Rz[3][3] =
    {
        { std::cos(yaw), -std::sin(yaw), 0 },
        { std::sin(yaw), std::cos(yaw), 0 },
        { 0, 0, 1 }
    };

    double Rtmp[3][3];
    multiplyRotationMatrices(Ry, Rx, Rtmp);
    multiplyRotationMatrices(Rz, Rtmp, Ro);
}

void BnUtils::blender_euler_to_rotation_matrix_degree(double roll, double pitch, double yaw, double (* const Ro)[3]) {
    blender_euler_to_rotation_matrix_rad(
        toRadians(roll),
        toRadians(pitch),
        toRadians(yaw),
        Ro );
}

BnQuaternion BnUtils::createQuanternion(BnAxisConfig const &axisConfig, double const * const values) {
    double valuesTmp[4] = { values[0], values[1], values[2], values[3] };
    axisConfig.apply(valuesTmp);
    return BnQuaternion(valuesTmp);
}

// all vals are double[4]
void BnUtils::transformSensorQuat(
    double const * const sensorQuatVals,
    double * const firstQuatVals,
    double const * const startingQuatVals,
    double const * const envQuatVals,
    int const * const pureAxisConfig,
    double * const valuesOut ) {

    BnAxisConfig axisConfig;
    axisConfig.config(pureAxisConfig);

    BnQuaternion const sensorQuat(sensorQuatVals);
    BnQuaternion const startingQuat(startingQuatVals);
    BnQuaternion const envQuat(envQuatVals);

    BnQuaternion firstQuat(firstQuatVals);
    if ( firstQuat.isEmpty()) {
        firstQuat = sensorQuat.inverse();
        firstQuat.toList(firstQuatVals);
    }

    BnQuaternion const rotationRealQuat = firstQuat.mul( sensorQuat );
    double valuesTmp[4];
    rotationRealQuat.toList( valuesTmp );
    BnQuaternion const rotationRealaxisQuat = createQuanternion(
            axisConfig,
            valuesTmp);

    envQuat.mul(rotationRealaxisQuat.mul(envQuat.inverse().mul(startingQuat))).toList(valuesOut);
}

} //namespace common

} //namespace bodynodesdev