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

package eu.bodynodesdev.common;

import java.util.Arrays;

public class BnReorientAxis {
    private int[] mReorientAxis;
    private int[] mReorientSign;

    public BnReorientAxis() {
        mReorientAxis = new int[]{ 0, 1, 2, 3 };
        mReorientSign = new int[]{ 1, 1, 1, 1 };
    }
   
    public void config(int[] ioAxis, int[] ioSign) {
        mReorientAxis = Arrays.copyOf(ioAxis, ioAxis.length);
        mReorientSign = Arrays.copyOf(ioSign, ioSign.length);
    }

    public void apply(float[] iovalues ) {
        float[] ovalues = new float[iovalues.length];
        for( int idv = 0; idv < iovalues.length; ++idv ) {
            ovalues[idv] = iovalues[ mReorientAxis[idv] ] * mReorientSign[idv];
        }
        for( int idv = 0; idv < iovalues.length; ++idv ) {
            iovalues[idv] = ovalues[idv];
        }
    }

    public void apply(int[] iovalues ) {
        int[] ovalues = new int[iovalues.length];
        for( int idv = 0; idv < iovalues.length; ++idv ) {
            ovalues[idv] = iovalues[ mReorientAxis[idv] ] * mReorientSign[idv];
        }
        for( int idv = 0; idv < iovalues.length; ++idv ) {
            iovalues[idv] = ovalues[idv];
        }
    }
}


