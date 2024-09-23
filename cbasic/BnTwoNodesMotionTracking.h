/*
# MIT License
# 
# Copyright (c) 2024 Manuel Bottini
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
*/

#ifndef BN_TWO_NODES_MOTION_TRACKING_C
#define BN_TWO_NODES_MOTION_TRACKING_C

#include "stdint.h"

typedef struct BnTwoNodesMotionTracking_st {
    float initialPosition[3];
    float lengthArm1;
    float lengthArm2;
    float locationConstraints[6];
    uint8_t hasLocationConstraints;
} BnTwoNodesMotionTracking_t;

#ifdef __cplusplus
extern "C" {
#endif

// *locationConstraints => locationConstraints[6] = { minX, maxX, minY, maxY, minZ, maxZ };
void BnTwoNodesMotionTracking_create(BnTwoNodesMotionTracking_t *data,
    float const initialPosition[3], float const lengthArm1, float const lengthArm2,
    float const *locationConstraints, char const *units);

void BnTwoNodesMotionTracking_compute( BnTwoNodesMotionTracking_t const *data,
    float const node1Quat[4], float const node2Quat[4], float finalPosition[3] );

#ifdef __cplusplus
}
#endif

#endif // BN_TWO_NODES_MOTION_TRACKING_C

