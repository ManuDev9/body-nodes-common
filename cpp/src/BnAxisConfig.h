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

#ifndef BN_AXIS_CONFIG_H
#define BN_AXIS_CONFIG_H

#define MAX_NUMBER_AXIS 4

#include <cstdint>

namespace bodynodesdev {

namespace common {

class BnAxisConfig {

  public:
    BnAxisConfig();

    void config(int const *const ioAxis, int const *const ioSign);
    void config(int const *const pureArray);
    void config(int const ioAxisW, int const ioAxisX, int const ioAxisY, int const ioAxisZ, int const ioSignW,
                int const ioSignX, int const ioSignY, int const ioSignZ);

    void apply(double iovalues[]) const;
    void apply(float iovalues[]) const;
    void apply(int iovalues[]) const;

  private:
    int mAxis[MAX_NUMBER_AXIS];
    int mSign[MAX_NUMBER_AXIS];
    uint8_t mLength;
};

} // namespace common

} // namespace bodynodesdev

#endif // BN_AXIS_CONFIG_H
