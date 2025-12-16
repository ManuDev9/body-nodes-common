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

#ifndef BN_QUATERNION_H
#define BN_QUATERNION_H

#include "stdint.h"
#include <stddef.h>

#define BN_QUATERNION_BUFFER_SIZE 70

typedef struct BnQuaternion_st {
    double w;
    double x;
    double y;
    double z;
} BnQuaternion_t;

#ifdef __cplusplus
extern "C" {
#endif

BnQuaternion_t BnQuaternion_create_array(double const vals[4]);
BnQuaternion_t BnQuaternion_create_wxyz(double const w, double const x, double const y, double const z);
BnQuaternion_t BnQuaternion_mul(BnQuaternion_t const *const quat1, BnQuaternion_t const *const quat2);
BnQuaternion_t BnQuaternion_div(BnQuaternion_t const *const quatIn, double const scalar);
BnQuaternion_t BnQuaternion_conjugate(BnQuaternion_t const *const quatIn);
double BnQuaternion_norm(BnQuaternion_t const *const quat);
BnQuaternion_t BnQuaternion_inverse(BnQuaternion_t const *const quatIn);
int BnQuaternion_to_string(BnQuaternion_t const *const quat, char *const buffer, size_t const buffer_size);
void BnQuaternion_to_list(BnQuaternion_t const *const quat, double valsOut[4]);
uint8_t BnQuaternion_is_empty(BnQuaternion_t const *const quat);

#ifdef __cplusplus
}
#endif

#endif // BN_QUATERNION_H
