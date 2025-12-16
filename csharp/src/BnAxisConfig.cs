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

namespace BodynodesDev.Common
{
    public class BnAxisConfig
    {
        public BnAxisConfig()
        {
            mAxis = new int[] { 0, 1, 2, 3 };
            mSign = new int[] { 1, 1, 1, 1 };
        }

        public BnAxisConfig(int[] pureArray)
        {
            mSign = new int[] { pureArray[0], pureArray[1], pureArray[2], pureArray[3] };
            mAxis = new int[] { pureArray[4], pureArray[5], pureArray[6], pureArray[7] };
        }

        public BnAxisConfig(int signW, int signX, int signY, int signZ, int valW, int valX, int valY, int valZ)
        {
            mSign = new int[] { signW, signX, signY, signZ };
            mAxis = new int[] { valW, valX, valY, valZ };
        }

        public void Config(int[] ioAxis, int[] ioSign)
        {
            for (uint idv = 0; idv < ioAxis.Length; ++idv)
            {
                mAxis[idv] = ioAxis[idv];
                mSign[idv] = ioSign[idv];
            }
        }

        public void Apply(float[] iovalues)
        {
            float[] ovalues = new float[iovalues.Length];
            for (uint idv = 0; idv < iovalues.Length; ++idv)
            {
                ovalues[idv] = iovalues[mAxis[idv]] * mSign[idv];
            }
            for (uint idv = 0; idv < iovalues.Length; ++idv)
            {
                iovalues[idv] = ovalues[idv];
            }
        }

        public void Apply(double[] iovalues)
        {
            double[] ovalues = new double[iovalues.Length];
            for (uint idv = 0; idv < iovalues.Length; ++idv)
            {
                ovalues[idv] = iovalues[mAxis[idv]] * mSign[idv];
            }
            for (uint idv = 0; idv < iovalues.Length; ++idv)
            {
                iovalues[idv] = ovalues[idv];
            }
        }

        public void Apply(int[] iovalues)
        {
            int[] ovalues = new int[iovalues.Length];
            for (uint idv = 0; idv < iovalues.Length; ++idv)
            {
                ovalues[idv] = iovalues[mAxis[idv]] * mSign[idv];
            }
            for (uint idv = 0; idv < iovalues.Length; ++idv)
            {
                iovalues[idv] = ovalues[idv];
            }
        }

        private int[] mAxis;
        private int[] mSign;
    }
}

