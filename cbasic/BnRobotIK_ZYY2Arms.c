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

#include "BnRobotIK_ZYY2Arms.h"

#include "math.h"

const double BN_M_PI = 3.141592653589793;

void BnRobotIK_ZYY2Arms_create( BnRobotIK_ZYY2Arms_t *data,
        float const lengthRA2, float const lengthRA3, float const displSP[3], float const displEP[3], char const * unit ) {

    data->lengthRA2 = lengthRA2;
    data->lengthRA3 = lengthRA3;

    data->displSP[0] =  displSP[0];
    data->displSP[1] =  displSP[1];
    data->displSP[2] =  displSP[2];

    data->displEP[0] =  displEP[0];
    data->displEP[1] =  displEP[1];
    data->displEP[2] =  displEP[2];
}

void BnRobotIK_ZYY2Arms_compute( BnRobotIK_ZYY2Arms_t *data, float const endpoint[3], float outAngles[3] ) {
    
    float intermEP[3] = {
        endpoint[0] - data->displEP[0],
        endpoint[1] - data->displEP[1],
        endpoint[2] - data->displEP[2]
    };

    outAngles[0] = (float)( asin( ( intermEP[1] ) / sqrt( intermEP[0]*intermEP[0] + intermEP[1]*intermEP[1] ) ) );
    
    float diff_iSP_iEP[3] = {
        intermEP[0] - data->displSP[0],
        intermEP[1] - data->displSP[1],
        intermEP[2] - data->displSP[2]
    };

    float dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2];
    float dist_iSP_iEP = (float)(sqrt(dist_iSP_iEP_2) );
    float tmp = ( data->lengthRA2*data->lengthRA2 + dist_iSP_iEP_2 - data->lengthRA3*data->lengthRA3 ) / ( 2 * data->lengthRA2 * dist_iSP_iEP );
    
    float gamma_2_1 = (float)( acos(tmp) );
    float gamma_2_2 = (float)( asin( diff_iSP_iEP[2] / sqrt( diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[2] * diff_iSP_iEP[2] ) ) );
    outAngles[1] = -(gamma_2_1 + gamma_2_2);

    tmp = ( data->lengthRA3*data->lengthRA3 + data->lengthRA2*data->lengthRA2 - dist_iSP_iEP_2 ) / ( 2 * data->lengthRA3 * data->lengthRA2 );
    outAngles[2] = (float)(BN_M_PI - acos(tmp));
}
