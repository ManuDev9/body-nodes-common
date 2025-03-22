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

#ifndef BN_ROBOT_IK_ZYY_2ARMS_CPP
#define BN_ROBOT_IK_ZYY_2ARMS_CPP

#include <cstdint>
#include <cstring>
#include <string>

namespace bodynodesdev {

namespace common {
    
class BnRobotIK_ZYY2Arms {

    public:

        // Starting Point is assumed to be [0, 0, 0]
        BnRobotIK_ZYY2Arms( float const lengthRA2, float const lengthRA3, float const displSP[3], std::string const units );

        // The returned angles refer to the X axis
        void compute( float const endpoint[3], float outAngles[3] );

    private:

        float mLengthRA2;
        float mLengthRA3;
        float mDisplSP[3];

};

} //namespace common

} //namespace bodynodesdev

#endif // BN_ROBOT_IK_ZYY_2ARMS_CPP
