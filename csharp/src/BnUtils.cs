/**
* MIT License
* 
* Copyright (c) 2023-2025 Manuel Bottini
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

using System;

namespace BodynodesDev.Common
{

    public class BnUtils
    {

        // Prevent instantiation
        private BnUtils()
        {
            throw new NotSupportedException("This is a static class. It cannot be instantiated.");
        }


        public static double ToRadians(double degrees)
        {
            return degrees * (Math.PI / 180.0);
        }

        // Function to multiply two rotation matrices
        public static double[][] MultiplyRotationMatrices(double[][] R1, double[][] R2) {
            return new double[][] {
                new double[] {
                    R1[0][0] * R2[0][0] + R1[0][1] * R2[1][0] + R1[0][2] * R2[2][0],
                    R1[0][0] * R2[0][1] + R1[0][1] * R2[1][1] + R1[0][2] * R2[2][1],
                    R1[0][0] * R2[0][2] + R1[0][1] * R2[1][2] + R1[0][2] * R2[2][2]
                },
                new double[] {
                    R1[1][0] * R2[0][0] + R1[1][1] * R2[1][0] + R1[1][2] * R2[2][0],
                    R1[1][0] * R2[0][1] + R1[1][1] * R2[1][1] + R1[1][2] * R2[2][1],
                    R1[1][0] * R2[0][2] + R1[1][1] * R2[1][2] + R1[1][2] * R2[2][2]
                },
                new double[] {
                    R1[2][0] * R2[0][0] + R1[2][1] * R2[1][0] + R1[2][2] * R2[2][0],
                    R1[2][0] * R2[0][1] + R1[2][1] * R2[1][1] + R1[2][2] * R2[2][1],
                    R1[2][0] * R2[0][2] + R1[2][1] * R2[1][2] + R1[2][2] * R2[2][2]
                }
            };
        }

        // Function to multiply a rotation matrix and a vector
        public static double[] MultiplyRotationMatrixWithVector(double[][] R1, double[] V1) {
            return new double[]
                {
                    R1[0][0] * V1[0] + R1[0][1] * V1[1] + R1[0][2] * V1[2],
                    R1[1][0] * V1[0] + R1[1][1] * V1[1] + R1[1][2] * V1[2],
                    R1[2][0] * V1[0] + R1[2][1] * V1[1] + R1[2][2] * V1[2]
                };
        }

        // Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in radians
        public static double[][] blender_euler_to_rotation_matrix_rad(double roll, double pitch, double yaw) {
            // counter rh wise rotations
            double[][] R_x = new double[][]
            {
                new double[] { 1, 0, 0 },
                new double[] { 0, Math.Cos(roll), -Math.Sin(roll) },
                new double[] { 0, Math.Sin(roll), Math.Cos(roll) }
            };

            double[][] R_y = new double[][]{
                new double[] { Math.Cos(pitch), 0, Math.Sin(pitch) },
                new double[] { 0, 1, 0 },
                new double[] { -Math.Sin(pitch), 0, Math.Cos(pitch) }
            };

            double[][] R_z = new double[][]{
                new double[] { Math.Cos(yaw), -Math.Sin(yaw), 0},
                new double[] { Math.Sin(yaw), Math.Cos(yaw), 0},
                new double[] {0, 0, 1}
            };

            return MultiplyRotationMatrices(R_z, MultiplyRotationMatrices(R_y, R_x));
        }


        // Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in degrees
        public static double[][] blender_euler_to_rotation_matrix_degree(double roll, double pitch, double yaw) {
            return blender_euler_to_rotation_matrix_rad(ToRadians(roll), ToRadians(pitch), ToRadians(yaw));
        }

        public static BnQuaternion CreateQuanternion(BnAxisConfig axis_config, double[] values) {
            axis_config.Apply(values);
            return new BnQuaternion(values);
        }

        /** This trasformation ignores what is the local axis of the object once rotated, we don't trust those axis system which might be compromised
        * sensorQuatVals               - is the raw quaternion (vector of 4 values) from the sensor
        * firstQuatVals                - [inout] is the first quaternion that has been registered from the sensor, this will create a 0 angle from where the sensor started sensing. Please fill it with 0,0,0,0 to set it as initially empty
        * startingQuatVals             - is the starting quaternion of the object we want to rotated with the sensor
        * envQuatVals                  - is the environment quaternion, basically indicates somehow where the x axis points 
        * pureAxisConfig               - axis configuration that will transfor the sensor values into values for the virtual world. Composed of 8 int array values that I can use to create a BnAxisConfig
        */
        public static double[] TransformSensorQuat( double[] sensorQuatVals, double[] firstQuatVals, double[] startingQuatVals, double[] envQuatVals, int[] pureAxisConfig) {

            BnAxisConfig axisConfig = new BnAxisConfig(pureAxisConfig);

            BnQuaternion sensorQuat = new BnQuaternion(sensorQuatVals);
            BnQuaternion startingQuat = new BnQuaternion(startingQuatVals);
            BnQuaternion envQuat = new BnQuaternion(envQuatVals);

            BnQuaternion firstQuat = new BnQuaternion(firstQuatVals);
            if (firstQuat.IsEmpty()) {
                firstQuat = sensorQuat.Inverse();
                firstQuatVals[0] = firstQuat.GetW();
                firstQuatVals[1] = firstQuat.GetX();
                firstQuatVals[2] = firstQuat.GetY();
                firstQuatVals[3] = firstQuat.GetZ();
            }


            BnQuaternion rotationRealQuat = firstQuat.Mul(sensorQuat);
            BnQuaternion rotationRealaxisQuat = CreateQuanternion(
                    axisConfig,
                    rotationRealQuat.ToList());

            BnQuaternion objectNewQuat = envQuat.Mul(rotationRealaxisQuat.Mul(envQuat.Inverse().Mul(startingQuat)));
            return objectNewQuat.ToList();
        }



        public static bool DoesKeyContainBodypart(string key, string bodypart)
        {
            if (key[0] == bodypart[0] && key[1] == bodypart[1] && key[2] == bodypart[2] && key[3] == bodypart[3])
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public static bool DoesKeyContainPlayer(string key, string player)
        {
            if (key[4] == player[0] && key[5] == player[1] && key[6] == player[2] && key[7] == player[3])
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        // Buf is a 4 bytes char array
        public static string CreateBodypartKey(BnDatatypes.BnBodypart bodypart)
        {
            return new string(bodypart.value);
        }

        // Buf is a 4 bytes char array
        public static string CreatePlayerKey(BnDatatypes.BnPlayer player)
        {
            return new string(player.value);
        }

        // Buf is a 8 bytes char array
        public static string CreatePlayerBodypartKey(BnDatatypes.BnPlayer player, BnDatatypes.BnBodypart bodypart)
        {
            return new string(player.value +"_"+bodypart.value);
        }

        // Buf is a 12 bytes char array
        public static string CreatePlayerBodypartSensortypeKey(BnDatatypes.BnPlayer player, BnDatatypes.BnBodypart bodypart, BnDatatypes.BnSensorType sensortype)
        {
            return new string(player.value + "_" + bodypart.value + "_"  +sensortype.value);
        }
    }
}
