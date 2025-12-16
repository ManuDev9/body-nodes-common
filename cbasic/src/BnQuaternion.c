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

#include "BnQuaternion.h"

#include <math.h>
#include <stdio.h>

BnQuaternion_t BnQuaternion_create_array(double const vals[4]) {
    BnQuaternion_t data = {vals[0], vals[1], vals[2], vals[3]};
    return data;
}

BnQuaternion_t BnQuaternion_create_wxyz(double const w, double const x, double const y, double const z) {
    BnQuaternion_t data = {w, x, y, z};
    return data;
}

BnQuaternion_t BnQuaternion_mul(BnQuaternion_t const *const quat1, BnQuaternion_t const *const quat2) {
    // Quaternion multiplication
    BnQuaternion_t data = {quat1->w * quat2->w - quat1->x * quat2->x - quat1->y * quat2->y - quat1->z * quat2->z,
                           quat1->w * quat2->x + quat1->x * quat2->w + quat1->y * quat2->z - quat1->z * quat2->y,
                           quat1->w * quat2->y + quat1->y * quat2->w + quat1->z * quat2->x - quat1->x * quat2->z,
                           quat1->w * quat2->z + quat1->z * quat2->w + quat1->x * quat2->y - quat1->y * quat2->x};
    return data;
}

BnQuaternion_t BnQuaternion_div(BnQuaternion_t const *const quatIn, double const scalar) {
    BnQuaternion_t data = {quatIn->w / scalar, quatIn->x / scalar, quatIn->y / scalar, quatIn->z / scalar};
    return data;
}

BnQuaternion_t BnQuaternion_conjugate(BnQuaternion_t const *const quatIn) {
    BnQuaternion_t data = {quatIn->w, -quatIn->x, -quatIn->y, -quatIn->z};
    return data;
}

double BnQuaternion_norm(BnQuaternion_t const *const quat) {
    return sqrt(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
}

BnQuaternion_t BnQuaternion_inverse(BnQuaternion_t const *const quatIn) {
    double const norm_val = BnQuaternion_norm(quatIn);
    BnQuaternion_t quatTmp = BnQuaternion_conjugate(quatIn);
    return BnQuaternion_div(&quatTmp, norm_val * norm_val);
}

int BnQuaternion_to_string(BnQuaternion_t const *const quat, char *const buffer, size_t const buffer_size) {

    if (buffer_size < BN_QUATERNION_BUFFER_SIZE) {
        return -1; // Buffer too small
    }
    return snprintf(buffer, buffer_size, "BnQuaternion(w=%.5f, x=%.5f, y=%.5f, z=%.5f)", quat->w, quat->x, quat->y,
                    quat->z);
}

void BnQuaternion_to_list(BnQuaternion_t const *const quat, double valsOut[4]) {
    valsOut[0] = quat->w;
    valsOut[1] = quat->x;
    valsOut[2] = quat->y;
    valsOut[3] = quat->z;
}

uint8_t BnQuaternion_is_empty(BnQuaternion_t const *const quat) {
    return quat->w == 0.0 && quat->x == 0.0 && quat->y == 0.0 && quat->z == 0.0;
}
