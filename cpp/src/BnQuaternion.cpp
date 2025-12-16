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

#include "BnQuaternion.h"

#include <cmath>
#include <stdio.h>

namespace bodynodesdev {

namespace common {

BnQuaternion::BnQuaternion(double const vals[4]) : mW(vals[0]), mX(vals[1]), mY(vals[2]), mZ(vals[3]) {}
BnQuaternion::BnQuaternion(double const w, double const x, double const y, double const z)
    : mW(w), mX(x), mY(y), mZ(z) {}

BnQuaternion BnQuaternion::mul(BnQuaternion const &quat) const {
    // Quaternion multiplication
    return BnQuaternion(mW * quat.w() - mX * quat.x() - mY * quat.y() - mZ * quat.z(),
                        mW * quat.x() + mX * quat.w() + mY * quat.z() - mZ * quat.y(),
                        mW * quat.y() + mY * quat.w() + mZ * quat.x() - mX * quat.z(),
                        mW * quat.z() + mZ * quat.w() + mX * quat.y() - mY * quat.x());
}

BnQuaternion BnQuaternion::div(double const scalar) const {
    return BnQuaternion(mW / scalar, mX / scalar, mY / scalar, mZ / scalar);
}

BnQuaternion BnQuaternion::conjugate() const { return BnQuaternion(mW, -mX, -mY, -mZ); }

double BnQuaternion::norm() const { return std::sqrt(mW * mW + mX * mX + mY * mY + mZ * mZ); }

BnQuaternion BnQuaternion::inverse() const {
    double const norm_val = norm();
    return conjugate().div(norm_val * norm_val);
}

std::ostream &operator<<(std::ostream &os, const BnQuaternion &quat) {
    return (os << "(" << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << ")");
}

void BnQuaternion::toList(double valsOut[4]) const {
    valsOut[0] = mW;
    valsOut[1] = mX;
    valsOut[2] = mY;
    valsOut[3] = mZ;
}

bool BnQuaternion::isEmpty() const { return mW == 0.0 && mX == 0.0 && mY == 0.0 && mZ == 0.0; }

double BnQuaternion::w() const { return mW; }

double BnQuaternion::x() const { return mX; }

double BnQuaternion::y() const { return mY; }

double BnQuaternion::z() const { return mZ; }

} // namespace common

} // namespace bodynodesdev
