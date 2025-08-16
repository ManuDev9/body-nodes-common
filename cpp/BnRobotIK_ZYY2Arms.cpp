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

#include "BnRobotIK_ZYY2Arms.h"

#include <cmath>


const double BN_M_PI = 3.141592653589793;

namespace bodynodesdev {

namespace common {

BnRobotIK_ZYY2Arms::BnRobotIK_ZYY2Arms(
        float const lengthRA2, float const lengthRA3, float const displSP[3], std::string const unit ) {
    mLengthRA2 = lengthRA2;
    mLengthRA3 = lengthRA3;
    memcpy( mDisplSP, displSP, sizeof(float) * 3 );
}

void BnRobotIK_ZYY2Arms::compute( float const endpoint[3], float outAngles[3] ) {

    outAngles[0] = static_cast<float>( std::asin( ( endpoint[1] ) / std::sqrt( endpoint[0]*endpoint[0] + endpoint[1]*endpoint[1] ) ) );
    
    float diff_iSP_iEP[3] = {
        endpoint[0] - mDisplSP[0],
        endpoint[1] - mDisplSP[1],
        endpoint[2] - mDisplSP[2]
    };

    float dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2];
    float dist_iSP_iEP = static_cast<float>(std::sqrt(dist_iSP_iEP_2) );
    float tmp = ( mLengthRA2*mLengthRA2 + dist_iSP_iEP_2 - mLengthRA3*mLengthRA3 ) / ( 2 * mLengthRA2 * dist_iSP_iEP );
    
    float gamma_2_1 = static_cast<float>( std::acos(tmp) );
    float gamma_2_2 = static_cast<float>( std::asin( diff_iSP_iEP[2] / std::sqrt( diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[2] * diff_iSP_iEP[2] ) ) );
    outAngles[1] = -(gamma_2_1 + gamma_2_2);

    tmp = ( mLengthRA3*mLengthRA3 + mLengthRA2*mLengthRA2 - dist_iSP_iEP_2 ) / ( 2 * mLengthRA3 * mLengthRA2 );
    outAngles[2] = static_cast<float>(BN_M_PI - std::acos(tmp));
}

} //namespace common

} //namespace bodynodesdev
