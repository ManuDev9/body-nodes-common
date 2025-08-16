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

#include "BnReorientAxis.h"

namespace bodynodesdev {

namespace common {

BnReorientAxis::BnReorientAxis() : mReorientAxis{ 0, 1, 2, 3 }, mReorientSign{ 1, 1, 1, 1}, mLength(0) {}

void BnReorientAxis::config( int const ioAxis[], int const ioSign[], uint8_t const length ) {
    mLength = length;
    for( uint8_t idl = 0; idl < mLength; ++idl ) {
        mReorientAxis[idl] = ioAxis[idl];
        mReorientSign[idl] = ioSign[idl];
    }
}

void BnReorientAxis::apply( float iovalues[] ) {
    float ovalues[mLength];
    for( uint8_t idv = 0; idv < mLength; ++idv ) {
        ovalues[idv] = iovalues[ mReorientAxis[idv] ] * mReorientSign[idv];
    }
    for( uint8_t idv = 0; idv < mLength; ++idv ) {
        iovalues[idv] = ovalues[idv];
    }
}

void BnReorientAxis::apply( int iovalues[] ) {
    int ovalues[mLength];
    for( uint8_t idv = 0; idv < mLength; ++idv ) {
        ovalues[idv] = iovalues[ mReorientAxis[idv] ] * mReorientSign[idv];
    }
    for( uint8_t idv = 0; idv < mLength; ++idv ) {
        iovalues[idv] = ovalues[idv];
    }
}

} //namespace common

} //namespace bodynodesdev


