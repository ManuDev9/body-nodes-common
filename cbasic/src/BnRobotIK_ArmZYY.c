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

#include "BnRobotIK_ArmZYY.h"
#include "BnConstants.h"

#include "math.h"
#include <string.h>

static inline double clamp(double x, double lo, double hi) {
    return x < lo ? lo : x > hi ? hi : x;
}

BnRobotIK_ArmZYY_t BnRobotIK_ArmZYY_create(
    double const lengthRA1,
    double const lengthRA2,
    double const lengthRA3,
    double const (* const anglesConstraints)[2],
    char const * const units ) {

    double tmpAnglesConstraints[3][2] = {{0,0},{0,0},{0,0}};
    uint8_t tmpHasAnglesConstraints = 0;
    if (anglesConstraints != NULL ) {
        tmpHasAnglesConstraints = 1;
        tmpAnglesConstraints[0][0] = anglesConstraints[0][0];
        tmpAnglesConstraints[0][1] = anglesConstraints[0][1];
        tmpAnglesConstraints[1][0] = anglesConstraints[1][0];
        tmpAnglesConstraints[1][1] = anglesConstraints[1][1];
        tmpAnglesConstraints[2][0] = anglesConstraints[2][0];
        tmpAnglesConstraints[2][1] = anglesConstraints[2][1];
    }
    BnRobotIK_ArmZYY_t data = {
        .theta_RA1 = 0,
        .gamma_RA2 = 0,
        .gamma_RA3 = 0,
        .lengthRA1 = lengthRA1,
        .lengthRA2 = lengthRA2,
        .lengthRA3 = lengthRA3,
        .anglesConstraints = { 
            { tmpAnglesConstraints[0][0], tmpAnglesConstraints[0][1] },
            { tmpAnglesConstraints[1][0], tmpAnglesConstraints[1][1] },
            { tmpAnglesConstraints[2][0], tmpAnglesConstraints[2][1] }
        },
        .hasAnglesConstraints = tmpHasAnglesConstraints
    };
    strncpy(data.units, units, UNITS_BUFF_SIZE-1);
    data.units[UNITS_BUFF_SIZE-1] = '\0'; // safety measure
    return data;
}

void BnRobotIK_ArmZYY_compute(
    BnRobotIK_ArmZYY_t * const data,
    double const endpoint[3],
    double outAngles[3][3] ) {

    // First let'z find the Z rotation of Arm1 [ x, y, z ]
    if(endpoint[0] == 0 && endpoint[1] == 0) {
        // Z rotation is not defined if x,y is at the origin, we will return Nan
        data->theta_RA1 = NAN;
    } else {
        data->theta_RA1 = atan2(endpoint[1], endpoint[0]);
    }

    // Remove Arm1 from the equation, and let's find the endpoint coordinates from Arm2 as origin
    double const diff_iSP_iEP[3] = {
        endpoint[0],
        endpoint[1],
        endpoint[2] - data->lengthRA1
    };

    // Length of diff_iSP_iEP
    double const dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2];
    double const dist_iSP_iEP = sqrt(dist_iSP_iEP_2);

    double tmp = 0;

    // From the low of cosine
    if(dist_iSP_iEP == 0){
        data->gamma_RA2 = 0;
    } else {
        tmp = ( data->lengthRA2*data->lengthRA2 + dist_iSP_iEP_2 - data->lengthRA3*data->lengthRA3 ) / ( 2 * data->lengthRA2 * dist_iSP_iEP );

        // Let's try to get the right angle even though the point is unreachable 
        tmp =clamp(tmp, -1, 1);
        //TODO evaluate what happens in these scenarios
        
        // Internal angle start of Arm2 of the triangle Arm2+Arm3+endpoint
        double const gamma_2_1 = acos(tmp);
        // Angle of endpoint and origin Arm2
        double gamma_2_2 = 0;
        if (dist_iSP_iEP_2 != 0){
            gamma_2_2 =asin( diff_iSP_iEP[2] / sqrt( dist_iSP_iEP_2 ) );
        }
            
        // Angle from the global Z axis and local Z axis of Arm2
        data->gamma_RA2 = BN_M_PI/2-(gamma_2_1 + gamma_2_2);
    }
            
    double dist_iIP_iEP = data->lengthRA3;
    double dist_iIP_iEP_2 = dist_iIP_iEP*dist_iIP_iEP;
    if(data->hasAnglesConstraints == 1 ) {
        if (data->gamma_RA2 < data->anglesConstraints[1][0]){
            data->gamma_RA2 = data->anglesConstraints[1][0];
        } else if (data->gamma_RA2 > data->anglesConstraints[1][1] ){
            data->gamma_RA2 = data->anglesConstraints[1][1];
        }

        // Let's recalculate where my intermediate point is in case that the constraints kicked in
        double const iIP[3] = {
            data->lengthRA2*sin(data->gamma_RA2)*cos(data->theta_RA1),
            data->lengthRA2*sin(data->gamma_RA2)*sin(data->theta_RA1),
            data->lengthRA2*cos(data->gamma_RA2)
        };
        double const diff_iIP_iEP[3] = {
            diff_iSP_iEP[0]-iIP[0],
            diff_iSP_iEP[1]-iIP[1],
            diff_iSP_iEP[2]-iIP[2]
        };
        dist_iIP_iEP_2 = diff_iIP_iEP[0] * diff_iIP_iEP[0] + diff_iIP_iEP[1] * diff_iIP_iEP[1] + diff_iIP_iEP[2] * diff_iIP_iEP[2];
        dist_iIP_iEP = sqrt(dist_iIP_iEP_2);
    }

    tmp = ( dist_iIP_iEP_2 + data->lengthRA2*data->lengthRA2 - dist_iSP_iEP_2 ) / ( 2 * dist_iIP_iEP * data->lengthRA2 );
    // Let's try to get the right angle even though the point is unreachable 
    tmp =clamp(tmp, -1, 1);
    // Internal angle start of Arm3 of the triangle Arm2+Arm3+endpoint, then changed to get the local angle we want
    data->gamma_RA3 = BN_M_PI - acos(tmp);

    if(data->hasAnglesConstraints == 1) {
        if ( !isnan(data->theta_RA1) ){
            if (data->theta_RA1 < data->anglesConstraints[0][0]) {
                data->theta_RA1 = data->anglesConstraints[0][0];
            } else if (data->theta_RA1 > data->anglesConstraints[0][1]) {
                data->theta_RA1 = data->anglesConstraints[0][1];
            }
        }

        if(data->gamma_RA3 < data->anglesConstraints[2][0]) {
            data->gamma_RA3 = data->anglesConstraints[2][0];
        } else if (data->gamma_RA3 > data->anglesConstraints[2][1]) {
            data->gamma_RA3 = data->anglesConstraints[2][1];
        }
    }

    outAngles[0][0] = 0;
    outAngles[0][1] = 0;
    outAngles[0][2] = data->theta_RA1;
    outAngles[1][0] = 0;
    outAngles[1][1] = data->gamma_RA2;
    outAngles[1][2] = 0;
    outAngles[2][0] = 0;
    outAngles[2][1] = data->gamma_RA3;
    outAngles[2][2] = 0;
}

void BnRobotIK_ArmZYY_getEndpoints(
    BnRobotIK_ArmZYY_t * const data,
    double endpoints[3][3]) {

    // Get the endpoint of Arm1, Arm2, and Arm3 from last compute() step
    endpoints[0][0] = 0;
    endpoints[0][1] = 0;
    endpoints[0][2] = data->lengthRA1;

    endpoints[1][0] = data->lengthRA2 * sin(data->gamma_RA2) * cos(data->theta_RA1);
    endpoints[1][1] = data->lengthRA2 * sin(data->gamma_RA2) * sin(data->theta_RA1);
    endpoints[1][2] = endpoints[0][2] + (data->lengthRA2 * cos(data->gamma_RA2));

    endpoints[2][0] = endpoints[1][0] + (data->lengthRA3 * sin(data->gamma_RA2+data->gamma_RA3) * cos(data->theta_RA1));
    endpoints[2][1] = endpoints[1][1] + (data->lengthRA3 * sin(data->gamma_RA2+data->gamma_RA3) * sin(data->theta_RA1));
    endpoints[2][2] = endpoints[1][2] + (data->lengthRA3 * cos(data->gamma_RA2+data->gamma_RA3));
}
