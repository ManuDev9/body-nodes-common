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

#include "BnRobotArm_MT.h"

namespace bodynodesdev {

namespace common {

BnRobotArm_MT::BnRobotArm_MT(std::unique_ptr<BnMotionTracking_Interface> &&motionTraker,
                             std::unique_ptr<BnRobotIK_Interface> &&robotIK)
    : motionTraker(std::move(motionTraker)), robotIK(std::move(robotIK)) {}

//    double const node1Quat[4],
//    double const node2Quat[4],
//    double endpositions[3][3],
//    double outAngles[3][3]
void BnRobotArm_MT::compute(double const *const node1Quat, double const *const node2Quat,
                            double (*const endpositions)[3], double (*const outAngles)[3]) {

    motionTraker->compute(node1Quat, node2Quat, endpositions);
    robotIK->compute(endpositions[2], outAngles);
}

void BnRobotArm_MT::setMotionTraker(std::unique_ptr<BnMotionTracking_Interface> &&mt) { motionTraker = std::move(mt); }

void BnRobotArm_MT::setRobotIK(std::unique_ptr<BnRobotIK_Interface> &&rik) { robotIK = std::move(rik); }

} // namespace common

} // namespace bodynodesdev
