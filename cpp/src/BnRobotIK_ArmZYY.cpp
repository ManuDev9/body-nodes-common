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

#include "BnRobotIK_ArmZYY.h"
#include "BnConstants.h"

#include <cmath>
#include <string.h>

namespace bodynodesdev {

namespace common {

static inline double clamp(double x, double lo, double hi) {
    return x < lo ? lo : x > hi ? hi : x;
}

BnRobotIK_ArmZYY::BnRobotIK_ArmZYY(
    double const lengthRA1,
    double const lengthRA2,
    double const lengthRA3,
    double const (* const anglesConstraints)[2],
    char const * const units )
    :
    mTheta_RA1(0),
    mGamma_RA2(0),
    mGamma_RA3(0),
    mLengthRA1(lengthRA1),
    mLengthRA2(lengthRA2),
    mLengthRA3(lengthRA3) {

    mHasAnglesConstraints = false;
    if (anglesConstraints != nullptr ) {
        mHasAnglesConstraints = true;
        memcpy(mAnglesConstraints, anglesConstraints, 6*sizeof(double));
    }

    strncpy(mUnits, units, UNITS_BUFF_SIZE-1);
    mUnits[UNITS_BUFF_SIZE-1] = '\0'; // safety measure
}

void BnRobotIK_ArmZYY::compute(
    double const * const endpoint,
    double (* const outAngles)[3] )  {

    // First let'z find the Z rotation of Arm1 [ x, y, z ]
    if(endpoint[0] == 0 && endpoint[1] == 0) {
        // Z rotation is not defined if x,y is at the origin, we will return Nan
        mTheta_RA1 = std::nan("");
    } else {
        mTheta_RA1 = std::atan2(endpoint[1], endpoint[0]);
    }

    // Remove Arm1 from the equation, and let's find the endpoint coordinates from Arm2 as origin
    double const diff_iSP_iEP[3] = {
        endpoint[0],
        endpoint[1],
        endpoint[2] - mLengthRA1
    };

    // Length of diff_iSP_iEP
    double const dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2];
    double const dist_iSP_iEP = std::sqrt(dist_iSP_iEP_2);

    double tmp = 0;

    // From the low of cosine
    if(dist_iSP_iEP == 0){
        mGamma_RA2 = 0;
    } else {
        tmp = ( mLengthRA2*mLengthRA2 + dist_iSP_iEP_2 - mLengthRA3*mLengthRA3 ) / ( 2 * mLengthRA2 * dist_iSP_iEP );

        // Let's try to get the right angle even though the point is unreachable 
        tmp = clamp(tmp, -1, 1);
        //TODO evaluate what happens in these scenarios
        
        // Internal angle start of Arm2 of the triangle Arm2+Arm3+endpoint
        double const gamma_2_1 = std::acos(tmp);
        // Angle of endpoint and origin Arm2
        double gamma_2_2 = 0;
        if (dist_iSP_iEP_2 != 0){
            gamma_2_2 =std::asin( diff_iSP_iEP[2] / std::sqrt( dist_iSP_iEP_2 ) );
        }
            
        // Angle from the global Z axis and local Z axis of Arm2
        mGamma_RA2 = BN_M_PI/2-(gamma_2_1 + gamma_2_2);
    }
            
    double dist_iIP_iEP = mLengthRA3;
    double dist_iIP_iEP_2 = dist_iIP_iEP*dist_iIP_iEP;
    if(mHasAnglesConstraints == 1 ) {
        if (mGamma_RA2 < mAnglesConstraints[1][0]){
            mGamma_RA2 = mAnglesConstraints[1][0];
        } else if (mGamma_RA2 > mAnglesConstraints[1][1] ){
            mGamma_RA2 = mAnglesConstraints[1][1];
        }

        // Let's recalculate where my intermediate point is in case that the constraints kicked in
        double const iIP[3] = {
            mLengthRA2*std::sin(mGamma_RA2)*std::cos(mTheta_RA1),
            mLengthRA2*std::sin(mGamma_RA2)*std::sin(mTheta_RA1),
            mLengthRA2*std::cos(mGamma_RA2)
        };
        double const diff_iIP_iEP[3] = {
            diff_iSP_iEP[0]-iIP[0],
            diff_iSP_iEP[1]-iIP[1],
            diff_iSP_iEP[2]-iIP[2]
        };
        dist_iIP_iEP_2 = diff_iIP_iEP[0] * diff_iIP_iEP[0] + diff_iIP_iEP[1] * diff_iIP_iEP[1] + diff_iIP_iEP[2] * diff_iIP_iEP[2];
        dist_iIP_iEP = std::sqrt(dist_iIP_iEP_2);
    }

    tmp = ( dist_iIP_iEP_2 + mLengthRA2*mLengthRA2 - dist_iSP_iEP_2 ) / ( 2 * dist_iIP_iEP * mLengthRA2 );
    // Let's try to get the right angle even though the point is unreachable 
    tmp = clamp(tmp, -1, 1);
    // Internal angle start of Arm3 of the triangle Arm2+Arm3+endpoint, then changed to get the local angle we want
    mGamma_RA3 = BN_M_PI - std::acos(tmp);

    if(mHasAnglesConstraints == 1) {
        if ( !std::isnan(mTheta_RA1) ) {
            if (mTheta_RA1 < mAnglesConstraints[0][0]) {
                mTheta_RA1 = mAnglesConstraints[0][0];
            } else if (mTheta_RA1 > mAnglesConstraints[0][1]) {
                mTheta_RA1 = mAnglesConstraints[0][1];
            }
        }

        if(mGamma_RA3 < mAnglesConstraints[2][0]) {
            mGamma_RA3 = mAnglesConstraints[2][0];
        } else if (mGamma_RA3 > mAnglesConstraints[2][1]) {
            mGamma_RA3 = mAnglesConstraints[2][1];
        }
    }

    outAngles[0][0] = 0;
    outAngles[0][1] = 0;
    outAngles[0][2] = mTheta_RA1;
    outAngles[1][0] = 0;
    outAngles[1][1] = mGamma_RA2;
    outAngles[1][2] = 0;
    outAngles[2][0] = 0;
    outAngles[2][1] = mGamma_RA3;
    outAngles[2][2] = 0;
}

void BnRobotIK_ArmZYY::getEndpoints(
    double endpoints[3][3]) const {

    // Get the endpoint of Arm1, Arm2, and Arm3 from last compute() step
    endpoints[0][0] = 0;
    endpoints[0][1] = 0;
    endpoints[0][2] = mLengthRA1;

    endpoints[1][0] = mLengthRA2 * sin(mGamma_RA2) * cos(mTheta_RA1);
    endpoints[1][1] = mLengthRA2 * sin(mGamma_RA2) * sin(mTheta_RA1);
    endpoints[1][2] = endpoints[0][2] + (mLengthRA2 * cos(mGamma_RA2));

    endpoints[2][0] = endpoints[1][0] + (mLengthRA3 * sin(mGamma_RA2+mGamma_RA3) * cos(mTheta_RA1));
    endpoints[2][1] = endpoints[1][1] + (mLengthRA3 * sin(mGamma_RA2+mGamma_RA3) * sin(mTheta_RA1));
    endpoints[2][2] = endpoints[1][2] + (mLengthRA3 * cos(mGamma_RA2+mGamma_RA3));
}

BnRobotIK_Interface* BnRobotIK_ArmZYY::clone() const {
    return new BnRobotIK_ArmZYY(*this);
}

} //namespace common

} //namespace bodynodesdev
