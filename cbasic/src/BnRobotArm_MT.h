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

#include "BnRobotIK_ArmZYY.h"
#include "BnMotionTracking_2Nodes.h"

#ifndef BN_ROBOT_ARM_MT_H
#define BN_ROBOT_ARM_MT_H

typedef struct BnRobotArm_MT_ArmZYY_2Nodes_st {    
    BnMotionTracking_2Nodes_t motionTraker;
    BnRobotIK_ArmZYY_t robotIK;
} BnRobotArm_MT_ArmZYY_2Nodes_t;

// It contains common RobotIK and MotionTracking associations


BnRobotArm_MT_ArmZYY_2Nodes_t BnRobotArm_MT_ArmZYY_2Nodes_create(
    BnMotionTracking_2Nodes_t const * const motionTraker,
    BnRobotIK_ArmZYY_t const * const robotIK);


void BnRobotArm_MT_ArmZYY_2Nodes_compute(
    BnRobotArm_MT_ArmZYY_2Nodes_t * const robotMT,
    double const * const node1Quat,
    double const * const node2Quat,
    double (* const endpositions)[3],
    double (* const outAngles)[3]);

#endif // BN_ROBOT_ARM_MT_H
