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
    public class BnMotionTracking_2Nodes : BnMotionTracking_Interface {

        private double[] mInitialPosition;
        private double[] mArmVector1;
        private double[] mArmVector2;
        private double[][]? mLocationConstraints;
        private string mUnits;

        public BnMotionTracking_2Nodes(
              double[] initialPosition, double[] armVector1, double[] armVector2,
              double[][]? locationConstraints, string units ) {

            mInitialPosition = new double[initialPosition.Length];
            Array.Copy(initialPosition, mInitialPosition, initialPosition.Length);
            mArmVector1 = new double[armVector1.Length];
            Array.Copy(armVector1, mArmVector1, armVector1.Length);
            mArmVector2 = new double[armVector2.Length];
            Array.Copy(armVector2, mArmVector2, armVector2.Length);
            if( locationConstraints != null ){
                mLocationConstraints = new double[locationConstraints.Length][];
                for (int i = 0; i < locationConstraints.Length; i++) {
                    mLocationConstraints[i] = new double[locationConstraints[i].Length];
                    Array.Copy(locationConstraints[i], mLocationConstraints[i], locationConstraints[i].Length);
                }
            } else {
                mLocationConstraints = null;
            }
            mUnits = units;
        }
      
        private void quaternion_to_rotation_matrix( double[] quat, double[][] rotationMatrix ) {
            rotationMatrix[0][0] = 1 - 2*(quat[2]*quat[2] + quat[3]*quat[3]);
            rotationMatrix[0][1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]);
            rotationMatrix[0][2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]);

            rotationMatrix[1][0] = 2*(quat[1]*quat[2] + quat[0]*quat[3]);
            rotationMatrix[1][1] = 1 - 2*(quat[1]*quat[1] + quat[3]*quat[3]);
            rotationMatrix[1][2] = 2*(quat[2]*quat[3] - quat[0]*quat[1]);

            rotationMatrix[2][0] = 2*(quat[1]*quat[3] - quat[0]*quat[2]);
            rotationMatrix[2][1] = 2*(quat[2]*quat[3] + quat[0]*quat[1]);
            rotationMatrix[2][2] = 1 - 2*(quat[1]*quat[1] + quat[2]*quat[2]);
        }

        private void matrix_multiply_3x3( double[][] matrix, double[] vector, double[] result) {
            result[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2];
            result[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2];
            result[2] = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2];
        }

        public void Compute( double[] node1Quat, double[] node2Quat, double[][] endpositions) {

            double[] initialPosition = endpositions[0];
            double[] point1Position = endpositions[1];
            double[] point2Position = endpositions[2];

            double[][] node1RM = new double[3][]; 
            for (int i = 0; i < node1RM.Length; i++)
            {
                node1RM[i] = new double[3];
            }
            double[][] node2RM = new double[3][]; 
            for (int i = 0; i < node2RM.Length; i++)
            {
                node2RM[i] = new double[3];
            }
            quaternion_to_rotation_matrix(node1Quat, node1RM);
            quaternion_to_rotation_matrix(node2Quat, node2RM);

            double[] rotatedArm1 = new double[3];
            double[] rotatedArm2 = new double[3];
            matrix_multiply_3x3(node1RM, mArmVector1, rotatedArm1 );
            matrix_multiply_3x3(node2RM, mArmVector2, rotatedArm2 );

            initialPosition[0] = mInitialPosition[0];
            initialPosition[1] = mInitialPosition[1];
            initialPosition[2] = mInitialPosition[2];
            
            point1Position[0] = initialPosition[0] + rotatedArm1[0];
            point1Position[1] = initialPosition[1] + rotatedArm1[1];
            point1Position[2] = initialPosition[2] + rotatedArm1[2];

            point2Position[0] = point1Position[0] + rotatedArm2[0];
            point2Position[1] = point1Position[1] + rotatedArm2[1];
            point2Position[2] = point1Position[2] + rotatedArm2[2];

            if( mLocationConstraints != null ) {
                if( point2Position[0] < mLocationConstraints[0][0] ) {
                    point2Position[0] = mLocationConstraints[0][0];
                } else if( point2Position[0] > mLocationConstraints[0][1] ) {
                    point2Position[0] = mLocationConstraints[0][1];
                }

                if( point2Position[1] < mLocationConstraints[1][0] ) {
                    point2Position[1] = mLocationConstraints[1][0];
                } else if( point2Position[1] > mLocationConstraints[1][1] ) {
                    point2Position[1] = mLocationConstraints[1][1];
                }

                if( point2Position[2] < mLocationConstraints[2][0] ) {
                    point2Position[2] = mLocationConstraints[2][0];
                } else if( point2Position[2] > mLocationConstraints[2][1] ) {
                    point2Position[2] = mLocationConstraints[2][1];
                }
            }
        }
    }
}


