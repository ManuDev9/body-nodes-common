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

#ifndef BN_QUATERNION_H
#define BN_QUATERNION_H

#include "stdint.h"
#include <stddef.h>
#include <iostream>

namespace bodynodesdev {

namespace common {

class BnQuaternion {

    public:
        BnQuaternion( double const vals[4]);
        BnQuaternion( double const w, double const x, double const y, double const z );

        BnQuaternion mul( BnQuaternion const &quat2 ) const;
        BnQuaternion div( double const scalar ) const;
        BnQuaternion conjugate() const;
        double norm() const;
        BnQuaternion inverse() const;
        friend std::ostream& operator<<(std::ostream& os, const BnQuaternion& quat);
        void toList( double valsOut[4] ) const;
        bool isEmpty() const;

        double w() const;
        double x() const;
        double y() const;
        double z() const;

    private:
        double mW;
        double mX;
        double mY;
        double mZ;
};

} //namespace common

} //namespace bodynodesdev

#endif // BN_QUATERNION_H
