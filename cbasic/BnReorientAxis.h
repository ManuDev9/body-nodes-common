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

#ifndef BN_REORIENT_AXIS_C
#define BN_REORIENT_AXIS_C

#define MAX_NUMBER_AXIS 4

#include "stdint.h"

typedef struct BnReorientAxisData_st {
    int reorientAxis[MAX_NUMBER_AXIS];
    int reorientSign[MAX_NUMBER_AXIS];
    uint8_t length;
} BnReorientAxisData_t;

#ifdef __cplusplus
extern "C" {
#endif

void BnReorientAxis_config( BnReorientAxisData_t* data, int const ioAxis[], int const ioSign[], uint8_t const length );
void BnReorientAxis_apply_float( BnReorientAxisData_t* data, float iovalues[] );
void BnReorientAxis_apply_int( BnReorientAxisData_t* data, int iovalues[] );

#ifdef __cplusplus
}
#endif

#endif // BN_REORIENT_AXIS_C

