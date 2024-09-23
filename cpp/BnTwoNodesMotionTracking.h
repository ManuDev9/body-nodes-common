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

#ifndef BN_TWO_NODES_MOTION_TRACKING_CPP
#define BN_TWO_NODES_MOTION_TRACKING_CPP

#include <cstdint>
#include <cstring>
#include <string>

namespace bodynodesdev {

namespace common {

class BnTwoNodesMotionTracking {

    public:

        // *locationConstraints => locationConstraints[6] = { minX, maxX, minY, maxY, minZ, maxZ };
        BnTwoNodesMotionTracking(
            float const initialPosition[3], float const lengthArm1, float const lengthArm2,
            float const *locationConstraints, std::string const units);
        
        void compute( float const node1Quat[4], float const node2Quat[4], float finalPosition[3] );

    private:

        void quaternion_to_rotation_matrix( float const quat[4], float rotationMatrix[3][3] );
        void matrix_multiply_3x3( float const matrix[3][3], float const vector[3], float result[3]);

    private:

        float mInitialPosition[3];
        float mLengthArm1;
        float mLengthArm2;
        float mLocationConstraints[6];
        bool mHasLocationConstraints;

};

} //namespace common

} //namespace bodynodesdev

#endif // BN_TWO_NODES_MOTION_TRACKING_CPP

