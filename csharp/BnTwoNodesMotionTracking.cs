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

using System;

namespace BodynodesDev.Common
{
    public class BnTwoNodesMotionTracking
    {

        public BnTwoNodesMotionTracking(
                float[] initialPosition, float lengthArm1, float lengthArm2,
                float[,] locationConstraints, string units ) {
            
            mInitialPosition = new float[initialPosition.Length];
            Array.Copy(initialPosition, mInitialPosition, initialPosition.Length);
            
            mLengthArm1 = lengthArm1;
            mLengthArm2 = lengthArm2;

            mLocationConstraints = new float[locationConstraints.GetLength(0), locationConstraints.GetLength(1)];
            if( locationConstraints != null ) {
                Buffer.BlockCopy(locationConstraints, 0, mLocationConstraints, 0, sizeof(float) * locationConstraints.Length);
            } else {
                mLocationConstraints = null;
            }
        }

        private void quaternion_to_rotation_matrix( float[] quat, float[,] rotationMatrix ) {
            rotationMatrix[0, 0] = 1 - 2*(quat[2]*quat[2] + quat[3]*quat[3]);
            rotationMatrix[0, 1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
            rotationMatrix[0, 2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]);

            rotationMatrix[1, 0] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
            rotationMatrix[1, 1] = 1 - 2*(quat[1]*quat[1] + quat[3]*quat[3]);
            rotationMatrix[1, 2] = 2*(quat[2]*quat[3] - quat[0]*quat[1]);

            rotationMatrix[2, 0] = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
            rotationMatrix[2, 1] = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
            rotationMatrix[2, 2] = 1 - 2*(quat[1]*quat[1] + quat[2]*quat[2]);
        }

        private void matrix_multiply_3x3( float[,] matrix, float[] vector, float[] result) {
            result[0] = matrix[0, 0] * vector[0] + matrix[0, 1] * vector[1] + matrix[0, 2] * vector[2];
            result[1] = matrix[1, 0] * vector[0] + matrix[1, 1] * vector[1] + matrix[1, 2] * vector[2];
            result[2] = matrix[2, 0] * vector[0] + matrix[2, 1] * vector[1] + matrix[2, 2] * vector[2];
        }

        public void compute( float[] node1Quat, float[] node2Quat, float[] finalPosition ) {

            float[,] node1RM = new float[3, 3];
            float[,] node2RM = new float[3, 3];
            quaternion_to_rotation_matrix(node1Quat, node1RM);
            quaternion_to_rotation_matrix(node2Quat, node2RM);

            float[] rotatedArm1 = new float[3];
            float[] rotatedArm2 = new float[3];
            matrix_multiply_3x3(node1RM, new float[]{ mLengthArm1, 0, 0 }, rotatedArm1 );
            matrix_multiply_3x3(node2RM, new float[]{ mLengthArm2, 0, 0 }, rotatedArm2 );

            finalPosition[0] = mInitialPosition[0] + rotatedArm1[0] + rotatedArm2[0];
            finalPosition[1] = mInitialPosition[1] + rotatedArm1[1] + rotatedArm2[1];
            finalPosition[2] = mInitialPosition[2] + rotatedArm1[2] + rotatedArm2[2];

            if( mLocationConstraints != null ) {
                if( finalPosition[0] < mLocationConstraints[0, 0] ) {
                    finalPosition[0] = mLocationConstraints[0, 0];
                } else if( finalPosition[0] > mLocationConstraints[0, 1] ) {
                    finalPosition[0] = mLocationConstraints[0, 1];
                }

                if( finalPosition[1] < mLocationConstraints[1, 0] ) {
                    finalPosition[1] = mLocationConstraints[1, 0];
                } else if( finalPosition[1] > mLocationConstraints[1, 1] ) {
                    finalPosition[1] = mLocationConstraints[1, 1];
                }

                if( finalPosition[2] < mLocationConstraints[2, 0] ) {
                    finalPosition[2] = mLocationConstraints[2, 0];
                } else if( finalPosition[2] > mLocationConstraints[2, 1] ) {
                    finalPosition[2] = mLocationConstraints[2, 1];
                }
            }
        }

        private float[] mInitialPosition;
        private float mLengthArm1;
        private float mLengthArm2;
        private float[,]? mLocationConstraints; 
    }
}

