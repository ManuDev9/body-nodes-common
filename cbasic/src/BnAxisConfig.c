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

void BnAxisConfig_config(BnAxisConfig_t *const data, int const *const ioAxis,
                         int const *const ioSign) {
  data->length = 4;
  for (uint8_t idl = 0; idl < data->length; ++idl) {
    data->sign[idl] = ioSign[idl];
    data->axis[idl] = ioAxis[idl];
  }
}

// mSign = new int[]{ pureArray[0], pureArray[1], pureArray[2], pureArray[3] };
// mAxis = new int[]{ pureArray[4], pureArray[5], pureArray[6], pureArray[7] };
void BnAxisConfig_config_purearray(BnAxisConfig_t *const data,
                                   int const *const pureArray) {
  data->length = 4;
  for (uint8_t idl = 0; idl < data->length; ++idl) {
    data->sign[idl] = pureArray[idl];
    data->axis[idl] = pureArray[idl + 4];
  }
}

void BnAxisConfig_config_vals(BnAxisConfig_t *const data, int const ioSignW,
                              int const ioSignX, int const ioSignY,
                              int const ioSignZ, int const ioAxisW,
                              int const ioAxisX, int const ioAxisY,
                              int const ioAxisZ) {

  data->length = 4;
  data->sign[0] = ioSignW;
  data->axis[0] = ioAxisW;
  data->sign[1] = ioSignX;
  data->axis[1] = ioAxisX;
  data->sign[2] = ioSignY;
  data->axis[2] = ioAxisY;
  data->sign[3] = ioSignZ;
  data->axis[3] = ioAxisZ;
}

void BnAxisConfig_apply_float(BnAxisConfig_t const *const data,
                              float *const iovalues) {
  float ovalues[data->length];
  for (uint8_t idv = 0; idv < data->length; ++idv) {
    ovalues[idv] = iovalues[data->axis[idv]] * data->sign[idv];
  }
  for (uint8_t idv = 0; idv < data->length; ++idv) {
    iovalues[idv] = ovalues[idv];
  }
}

void BnAxisConfig_apply_double(BnAxisConfig_t const *const data,
                               double *const iovalues) {
  double ovalues[data->length];
  for (uint8_t idv = 0; idv < data->length; ++idv) {
    ovalues[idv] = iovalues[data->axis[idv]] * data->sign[idv];
  }
  for (uint8_t idv = 0; idv < data->length; ++idv) {
    iovalues[idv] = ovalues[idv];
  }
}

void BnAxisConfig_apply_int(BnAxisConfig_t const *const data,
                            int *const iovalues) {
  int ovalues[data->length];
  for (uint8_t idv = 0; idv < data->length; ++idv) {
    ovalues[idv] = iovalues[data->axis[idv]] * data->sign[idv];
  }
  for (uint8_t idv = 0; idv < data->length; ++idv) {
    iovalues[idv] = ovalues[idv];
  }
}
