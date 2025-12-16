/*
# MIT License
#
# Copyright (c) 2024-2025 Manuel Bottini
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

#ifndef BN_MOTION_TRACKING_2_NODES_H
#define BN_MOTION_TRACKING_2_NODES_H

#include "stdint.h"

#define UNITS_BUFF_SIZE 10

typedef struct BnMotionTracking_2Nodes_st {
    double initialPosition[3];
    double armVector1[3];
    double armVector2[3];
    double locationConstraints[3][2];
    uint8_t hasLocationConstraints;
    char units[UNITS_BUFF_SIZE];
} BnMotionTracking_2Nodes_t;

#ifdef __cplusplus
extern "C" {
#endif

//    *locationConstraints => locationConstraints[3][2] = { {minX, maxX}, {minY,
//    maxY}, {minZ, maxZ} }; double const initialPosition[3], double const
//    armVector1[3], double const armVector2[3], double const (* const
//    locationConstraints)[2], char const units[UNITS_BUFF_SIZE]);
BnMotionTracking_2Nodes_t BnMotionTracking_2Nodes_create(double const *const initialPosition,
                                                         double const *const armVector1, double const *const armVector2,
                                                         double const (*const locationConstraints)[2],
                                                         char const *const units);

//    double const node1Quat[4],
//    double const node2Quat[4],
//    endpositions[3][3] = { {ip_endposX, ip_endposY, ip_endposZ},
//    {arm1_endposX, arm1_endposY, arm1_endposZ}, {arm2_endposX, arm2_endposY,
//    arm2_endposZ}};
void BnMotionTracking_2Nodes_compute(BnMotionTracking_2Nodes_t *const data, double const *const node1Quat,
                                     double const *const node2Quat, double (*const endpositions)[3]);

#ifdef __cplusplus
}
#endif

#endif // BN_MOTION_TRACKING_2_NODES_H
