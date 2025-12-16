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

#include "BnMotionTracking_2Nodes.h"
#include "BnRobotIK_ArmZYY.h"
#include <memory>

#ifndef BN_ROBOT_ARM_MT_H
#define BN_ROBOT_ARM_MT_H

namespace bodynodesdev {

namespace common {

class BnRobotArm_MT {

  public:
    BnRobotArm_MT(std::unique_ptr<BnMotionTracking_Interface> &&motionTraker,
                  std::unique_ptr<BnRobotIK_Interface> &&robotIK);

    void compute(double const *const node1Quat, double const *const node2Quat, double (*const endpositions)[3],
                 double (*const outAngles)[3]);

    void setMotionTraker(std::unique_ptr<BnMotionTracking_Interface> &&mt);
    void setRobotIK(std::unique_ptr<BnRobotIK_Interface> &&rik);

  private:
    std::unique_ptr<BnMotionTracking_Interface> motionTraker;
    std::unique_ptr<BnRobotIK_Interface> robotIK;
};

} // namespace common

} // namespace bodynodesdev

#endif // BN_ROBOT_ARM_MT_H
