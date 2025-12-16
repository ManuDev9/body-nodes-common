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

#ifndef BN_ROBOT_IK_ARM_ZYY_H
#define BN_ROBOT_IK_ARM_ZYY_H

#include "stdint.h"
#include "BnRobotIK_Interface.h"

#define UNITS_BUFF_SIZE 10

namespace bodynodesdev {

namespace common {

class BnRobotIK_ArmZYY : public BnRobotIK_Interface {

    public:
        BnRobotIK_ArmZYY(
            double const lengthRA1,
            double const lengthRA2,
            double const lengthRA3,
            double const (* const anglesConstraints)[2],
            char const * const units );

        //    double const endpoint[3],
        //    double outAngles[3][3] 
        void compute(
            double const * const endpoint,
            double (* const outAngles)[3] ) override;

        //    double endpoints[3][3]
        void getEndpoints( double (*const endpoints)[3] ) const;

        BnRobotIK_Interface* clone() const override;

    private:
        double mTheta_RA1;
        double mGamma_RA2;
        double mGamma_RA3;
        double mLengthRA1;
        double mLengthRA2;
        double mLengthRA3;
        double mAnglesConstraints[3][2];
        bool mHasAnglesConstraints;
        char mUnits[UNITS_BUFF_SIZE];
};

} //namespace common

} //namespace bodynodesdev

#endif // BN_ROBOT_IK_ARM_ZYY_H
