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

public class BnRobotArmZYY_IK implements BnRobotIK_Interface {

    private double mTheta_RA1;
    private double mGamma_RA2;
    private double mGamma_RA3;
    private double mLengthRA1;
    private double mLengthRA2;
    private double mLengthRA3;
    private double[][] mAnglesConstraints;
    private String mUnits;

    // All angles are in radiants
    // This is an example of anglesConstraints
    // anglesConstraints = [ [ -math.pi/2, math.pi/2 ], [0 , math.pi/2], [0, math.pi/2 ] ]
    // Starting Point is assumed to be [0, 0, 0]
    public BnRobotArmZYY_IK( double lengthRA1, double lengthRA2, double lengthRA3, double[][] anglesConstraints, String units ) {
        mTheta_RA1 = 0;
        mGamma_RA2 = 0;
        mGamma_RA3 = 0;
        mLengthRA1 = lengthRA1;
        mLengthRA2 = lengthRA2;
        mLengthRA3 = lengthRA3;
        if( anglesConstraints != null ){
            mAnglesConstraints = new double[anglesConstraints.length][];
            for (int i = 0; i < anglesConstraints.length; i++) {
                mAnglesConstraints[i] = Arrays.copyOf(anglesConstraints[i], anglesConstraints[i].length);
            }
            mAnglesConstraints = Arrays.copyOf(anglesConstraints, anglesConstraints.length);
        } else {
            mAnglesConstraints = null;
        }
        mUnits = units;
    }


    // The returned angles are made to work along the X axis
    // x rotation alpha 
    // y rotation gamma 
    // z rotation theta 
    // endpoint is expected to be an array of 3 elements: [ x, y, z ]. It is an input parameter
    // outAngles is expected to be an array of 3 elements: [ theta_RA1, gamma_RA2, gamma_RA3 ]. It is the output paramater
    @Override
    public void compute( double[] endpoint, double[] outAngles ) {

        // First let'z find the Z rotation of Arm1 [ x, y, z ]
        if(endpoint[0] == 0 && endpoint[1] == 0) {
            // Z rotation is not defined if x,y is at the origin, we will return Nan
            mTheta_RA1 = Double.NaN;
        } else {
            mTheta_RA1 = Math.atan2(endpoint[1], endpoint[0]);
        }

        // Remove Arm1 from the equation, and let's find the endpoint coordinates from Arm2 as origin
        double[] diff_iSP_iEP = new double[] {
            endpoint[0],
            endpoint[1],
            endpoint[2] - mLengthRA1
        };

        // Length of diff_iSP_iEP
        double dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2];
        double dist_iSP_iEP = Math.sqrt(dist_iSP_iEP_2);

        double tmp = 0;
        // From the low of cosine
        if(dist_iSP_iEP == 0){
            mGamma_RA2 = 0;
        } else {
            tmp = ( mLengthRA2*mLengthRA2 + dist_iSP_iEP_2 - mLengthRA3*mLengthRA3 ) / ( 2 * mLengthRA2 * dist_iSP_iEP );
            // Let's try to get the right angle even though the point is unreachable 
            tmp = Math.min(Math.max( tmp, -1 ), 1);
            //TODO evaluate what happens in these scenarios
            
            // Internal angle start of Arm2 of the triangle Arm2+Arm3+endpoint
            double gamma_2_1 = Math.acos(tmp);
            // Angle of endpoint and origin Arm2
            double gamma_2_2 = 0;
            if (dist_iSP_iEP_2 != 0){
                gamma_2_2 = Math.asin( diff_iSP_iEP[2] / Math.sqrt( dist_iSP_iEP_2 ) );
            }
                
            // Angle from the global Z axis and local Z axis of Arm2
            mGamma_RA2 = Math.PI/2-(gamma_2_1 + gamma_2_2);
        }
        
        double dist_iIP_iEP = mLengthRA3;
        double dist_iIP_iEP_2 = dist_iIP_iEP*dist_iIP_iEP;
        if(mAnglesConstraints != null) {
            if (mGamma_RA2 < mAnglesConstraints[1][0]){
                mGamma_RA2 = mAnglesConstraints[1][0];
            } else if (mGamma_RA2 > mAnglesConstraints[1][1] ){
                mGamma_RA2 = mAnglesConstraints[1][1];
            }

            // Let's recalculate where my intermediate point is in case that the constraints kicked in
            double[] iIP = new double[] {
                mLengthRA2*Math.sin(mGamma_RA2)*Math.cos(mTheta_RA1),
                mLengthRA2*Math.sin(mGamma_RA2)*Math.sin(mTheta_RA1),
                mLengthRA2*Math.cos(mGamma_RA2)
            };
            double[] diff_iIP_iEP = new double[] {
                diff_iSP_iEP[0]-iIP[0],
                diff_iSP_iEP[1]-iIP[1],
                diff_iSP_iEP[2]-iIP[2]
            };
            dist_iIP_iEP_2 = diff_iIP_iEP[0] * diff_iIP_iEP[0] + diff_iIP_iEP[1] * diff_iIP_iEP[1] + diff_iIP_iEP[2] * diff_iIP_iEP[2];
            dist_iIP_iEP = Math.sqrt(dist_iIP_iEP_2);
        }

        tmp = ( dist_iIP_iEP_2 + mLengthRA2*mLengthRA2 - dist_iSP_iEP_2 ) / ( 2 * dist_iIP_iEP * mLengthRA2 );
        // Let's try to get the right angle even though the point is unreachable 
        tmp = Math.min(Math.max( tmp, -1 ), 1); 
        // Internal angle start of Arm3 of the triangle Arm2+Arm3+endpoint, then changed to get the local angle we want
        mGamma_RA3 = (Math.PI - Math.acos(tmp));

        if(mAnglesConstraints != null) {
            if ( !Double.isNaN(mTheta_RA1) ){
                if (mTheta_RA1 < mAnglesConstraints[0][0]) {
                    mTheta_RA1 = mAnglesConstraints[0][0];
                } else if (mTheta_RA1 > mAnglesConstraints[0][1]) {
                    mTheta_RA1 = mAnglesConstraints[0][1];
                }
            }

            if(mGamma_RA3 < mAnglesConstraints[2][0]) {
                mGamma_RA3 = mAnglesConstraints[2][0];
            } else if (mGamma_RA3 > mAnglesConstraints[2][1]) {
                mGamma_RA3 = mAnglesConstraints[2][1];
            }
        }

        outAngles[0] = mTheta_RA1;
        outAngles[1] = mGamma_RA2;
        outAngles[2] = mGamma_RA3;

    }


    // endpoint is expected to be an array of 3 arrays, each subarray is compased of 3 elements: [ x, y, z ]. It is an output parameter
    public void getEndpoints(double[][] endpoints) {
        // Get the endpoint of Arm1, Arm2, and Arm3 from last compute() step
        endpoints[0][0] = 0;
        endpoints[0][1] = 0;
        endpoints[0][2] = mLengthRA1;

        endpoints[1][0] = mLengthRA2 * Math.sin(mGamma_RA2) * Math.cos(mTheta_RA1);
        endpoints[1][1] = mLengthRA2 * Math.sin(mGamma_RA2) * Math.sin(mTheta_RA1);
        endpoints[1][2] = endpoints[0][2] + (mLengthRA2 * Math.cos(mGamma_RA2));

        endpoints[2][0] = endpoints[1][0] + (mLengthRA3 * Math.sin(mGamma_RA2+mGamma_RA3) * Math.cos(mTheta_RA1));
        endpoints[2][1] = endpoints[1][1] + (mLengthRA3 * Math.sin(mGamma_RA2+mGamma_RA3) * Math.sin(mTheta_RA1));
        endpoints[2][2] = endpoints[1][2] + (mLengthRA3 * Math.cos(mGamma_RA2+mGamma_RA3));

    }
}


