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

using System;

namespace BodynodesDev.Common
{
    public class BnRobotIK_ZYY2Arms
    {
        public BnRobotIK_ZYY2Arms(
                float lengthRA2, float lengthRA3, float[] displSP, string unit ) {
            
            mLengthRA2 = lengthRA2;
            mLengthRA3 = lengthRA3;

            mDisplSP = new float[displSP.Length];
            Array.Copy(displSP, mDisplSP, displSP.Length);
        }

        public void compute( float[] endpoint, float[] outAngles ) {

            outAngles[0] = (float) Math.Asin( ( endpoint[1] ) / Math.Sqrt( endpoint[0]*endpoint[0] + endpoint[1]*endpoint[1] ) );
        
            float[] diff_iSP_iEP = new float[3];
            diff_iSP_iEP[0] = endpoint[0] - mDisplSP[0];
            diff_iSP_iEP[1] = endpoint[1] - mDisplSP[1];
            diff_iSP_iEP[2] = endpoint[2] - mDisplSP[2];

            float dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2];
            float dist_iSP_iEP = (float) Math.Sqrt(dist_iSP_iEP_2);
            float tmp = ( mLengthRA2*mLengthRA2 + dist_iSP_iEP_2 - mLengthRA3*mLengthRA3 ) / ( 2 * mLengthRA2 * dist_iSP_iEP );
        
            float gamma_2_1 = (float) Math.Acos(tmp);
            float gamma_2_2 = (float) Math.Asin( diff_iSP_iEP[2] / Math.Sqrt( diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[2] * diff_iSP_iEP[2] ) );
            outAngles[1] = -(gamma_2_1 + gamma_2_2);

            tmp = ( mLengthRA3*mLengthRA3 + mLengthRA2*mLengthRA2 - dist_iSP_iEP_2 ) / ( 2 * mLengthRA3 * mLengthRA2 );
            outAngles[2] = (float) (Math.PI - Math.Acos(tmp));
        }

        private float mLengthRA2;
        private float mLengthRA3;
        private float[] mDisplSP;
    }
}

