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

#include "BnAxisConfig.h"

namespace bodynodesdev {

namespace common {

BnAxisConfig::BnAxisConfig() : mAxis{0, 1, 2, 3}, mSign{1, 1, 1, 1}, mLength(0) {}

void BnAxisConfig::config(int const *const ioAxis, int const *const ioSign) {
    mLength = 4;
    for (uint8_t idl = 0; idl < mLength; ++idl) {
        mSign[idl] = ioSign[idl];
        mAxis[idl] = ioAxis[idl];
    }
}

// mSign = new int[]{ pureArray[0], pureArray[1], pureArray[2], pureArray[3] };
// mAxis = new int[]{ pureArray[4], pureArray[5], pureArray[6], pureArray[7] };
void BnAxisConfig::config(int const *const pureArray) {
    mLength = 4;
    for (uint8_t idl = 0; idl < mLength; ++idl) {
        mSign[idl] = pureArray[idl];
        mAxis[idl] = pureArray[idl + 4];
    }
}

void BnAxisConfig::config(int const ioSignW, int const ioSignX, int const ioSignY, int const ioSignZ, int const ioAxisW,
                          int const ioAxisX, int const ioAxisY, int const ioAxisZ) {

    mLength = 4;
    mSign[0] = ioSignW;
    mAxis[0] = ioAxisW;
    mSign[1] = ioSignX;
    mAxis[1] = ioAxisX;
    mSign[2] = ioSignY;
    mAxis[2] = ioAxisY;
    mSign[3] = ioSignZ;
    mAxis[3] = ioAxisZ;
}

void BnAxisConfig::apply(float *const iovalues) const {
    float ovalues[mLength];
    for (uint8_t idv = 0; idv < mLength; ++idv) {
        ovalues[idv] = iovalues[mAxis[idv]] * mSign[idv];
    }
    for (uint8_t idv = 0; idv < mLength; ++idv) {
        iovalues[idv] = ovalues[idv];
    }
}

void BnAxisConfig::apply(double *const iovalues) const {
    double ovalues[mLength];
    for (uint8_t idv = 0; idv < mLength; ++idv) {
        ovalues[idv] = iovalues[mAxis[idv]] * mSign[idv];
    }
    for (uint8_t idv = 0; idv < mLength; ++idv) {
        iovalues[idv] = ovalues[idv];
    }
}

void BnAxisConfig::apply(int *const iovalues) const {
    int ovalues[mLength];
    for (uint8_t idv = 0; idv < mLength; ++idv) {
        ovalues[idv] = iovalues[mAxis[idv]] * mSign[idv];
    }
    for (uint8_t idv = 0; idv < mLength; ++idv) {
        iovalues[idv] = ovalues[idv];
    }
}

} // namespace common

} // namespace bodynodesdev
