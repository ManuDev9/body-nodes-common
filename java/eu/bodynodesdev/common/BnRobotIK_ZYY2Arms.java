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

package eu.bodynodesdev.common;

import java.util.Arrays;

public class BnRobotIK_ZYY2Arms {

    private float mLengthRA2;
    private float mLengthRA3;
    private float[] mDisplSP;
    private float[] mDisplEP;

    // Starting Point is assumed to be [0, 0, 0]
    public BnRobotIK_ZYY2Arms( float lengthRA2, float lengthRA3, float displSP[], float displEP[], String unit ) {
        mLengthRA2 = lengthRA2;
        mLengthRA3 = lengthRA3;
        mDisplSP = Arrays.copyOf( displSP, displSP.length );
        mDisplEP = Arrays.copyOf( displEP, displEP.length );
    }

    // The returned angles are made to work along the X axis
    public void compute( float endpoint[], float outAngles[] ) {
    
        float intermEP[] = new float[3];
        intermEP[0] = endpoint[0] - mDisplEP[0];
        intermEP[1] = endpoint[1] - mDisplEP[1];
        intermEP[2] = endpoint[2] - mDisplEP[2];

        outAngles[0] = (float) Math.asin( ( intermEP[1] ) / Math.sqrt( intermEP[0]*intermEP[0] + intermEP[1]*intermEP[1] ) );
        
        float diff_iSP_iEP[] = new float[3];
        diff_iSP_iEP[0] = intermEP[0] - mDisplSP[0];
        diff_iSP_iEP[1] = intermEP[1] - mDisplSP[1];
        diff_iSP_iEP[2] = intermEP[2] - mDisplSP[2];

        float dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2];
        float dist_iSP_iEP = (float) Math.sqrt(dist_iSP_iEP_2);
        float tmp = ( mLengthRA2*mLengthRA2 + dist_iSP_iEP_2 - mLengthRA3*mLengthRA3 ) / ( 2 * mLengthRA2 * dist_iSP_iEP );
        
        float gamma_2_1 = (float) Math.acos(tmp);
        float gamma_2_2 = (float) Math.asin( diff_iSP_iEP[2] / Math.sqrt( diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[2] * diff_iSP_iEP[2] ) );
        outAngles[1] = -(gamma_2_1 + gamma_2_2);

        tmp = ( mLengthRA3*mLengthRA3 + mLengthRA2*mLengthRA2 - dist_iSP_iEP_2 ) / ( 2 * mLengthRA3 * mLengthRA2 );
        outAngles[2] = (float) (Math.PI - Math.acos(tmp));
    }
}


