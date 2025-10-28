/*
 * MIT License
 *
 * Copyright (c) 2025 Manuel Bottini
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS"; WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package eu.bodynodesdev.common;

public class BnUtils {

    // Prevent instantiation
    private BnUtils() {
        throw new UnsupportedOperationException("Class cannot be instantiated");
    }

    // Function to multiply two rotation matrices
    public static double[][] multiplyRotationMatrices(double[][] R1, double[][] R2) {
        return new double[][] {
            {
                R1[0][0] * R2[0][0] + R1[0][1] * R2[1][0] + R1[0][2] * R2[2][0],
                R1[0][0] * R2[0][1] + R1[0][1] * R2[1][1] + R1[0][2] * R2[2][1],
                R1[0][0] * R2[0][2] + R1[0][1] * R2[1][2] + R1[0][2] * R2[2][2]
            },
            {
                R1[1][0] * R2[0][0] + R1[1][1] * R2[1][0] + R1[1][2] * R2[2][0],
                R1[1][0] * R2[0][1] + R1[1][1] * R2[1][1] + R1[1][2] * R2[2][1],
                R1[1][0] * R2[0][2] + R1[1][1] * R2[1][2] + R1[1][2] * R2[2][2]
            },
            {
                R1[2][0] * R2[0][0] + R1[2][1] * R2[1][0] + R1[2][2] * R2[2][0],
                R1[2][0] * R2[0][1] + R1[2][1] * R2[1][1] + R1[2][2] * R2[2][1],
                R1[2][0] * R2[0][2] + R1[2][1] * R2[1][2] + R1[2][2] * R2[2][2]
            }
        };
    }

    // Function to multiply a rotation matrix and a vector
    public static double[] multiplyRotationMatrixWithVector(double[][] R1, double[] V1) {
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
        final double[][] R_x = new double[][]{
            { 1, 0, 0 },
            { 0, Math.cos(roll), -Math.sin(roll)},
            { 0, Math.sin(roll), Math.cos(roll) }
        };

        final double[][] R_y = new double[][]{
            { Math.cos(pitch), 0, Math.sin(pitch) },
            { 0, 1, 0 },
            { -Math.sin(pitch), 0, Math.cos(pitch) }
        };

        final double[][] R_z = new double[][]{
            {Math.cos(yaw), -Math.sin(yaw), 0},
            { Math.sin(yaw), Math.cos(yaw), 0},
            {0, 0, 1}
        };

        return multiplyRotationMatrices(R_z, multiplyRotationMatrices(R_y, R_x));
    }


    // Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in degrees
    public static double[][] blender_euler_to_rotation_matrix_degree(double roll, double pitch, double yaw) {
        return blender_euler_to_rotation_matrix_rad(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw));
    }

    public static BnQuaternion createQuanternion(BnAxisConfig axis_config, double[] values) {
        return new BnQuaternion(
            axis_config.newWsign * values[axis_config.newWval],
            axis_config.newXsign * values[axis_config.newXval],
            axis_config.newYsign * values[axis_config.newYval],
            axis_config.newZsign * values[axis_config.newZval]
        );
    }

    /** This trasformation ignores what is the local axis of the object once rotated, we don't trust those axis system which might be compromised
    * sensorQuatVals               - is the raw quaternion (vector of 4 values) from the sensor
    * firstQuatVals                - [inout] is the first quaternion that has been registered from the sensor, this will create a 0 angle from where the sensor started sensing. Please fill it with 0,0,0,0 to set it as initially empty
    * startingQuatVals             - is the starting quaternion of the object we want to rotated with the sensor
    * envQuatVals                  - is the environment quaternion, basically indicates somehow where the x axis points 
    * pureAxisConfig               - axis configuration that will transfor the sensor values into values for the virtual world. Composed of 8 int array values that I can use to create a BnAxisConfig
    */
    public static double[] transformSensorQuat( double[] sensorQuatVals, double[] firstQuatVals, double[] startingQuatVals, double[] envQuatVals, int[] pureAxisConfig) {

        BnAxisConfig axisConfig = new BnAxisConfig(pureAxisConfig);

        BnQuaternion sensorQuat = new BnQuaternion(sensorQuatVals);
        BnQuaternion startingQuat = new BnQuaternion(startingQuatVals);
        BnQuaternion envQuat = new BnQuaternion(envQuatVals);

        BnQuaternion firstQuat = new BnQuaternion(firstQuatVals);
        if (firstQuat.isEmpty()) {
            firstQuat = sensorQuat.inverse();
            firstQuatVals[0] = firstQuat.getW();
            firstQuatVals[1] = firstQuat.getX();
            firstQuatVals[2] = firstQuat.getY();
            firstQuatVals[3] = firstQuat.getZ();
        }

        BnQuaternion rotationRealQuat = firstQuat.mul(sensorQuat);
        BnQuaternion rotationRealaxisQuat = createQuanternion(
                axisConfig,
                rotationRealQuat.toList());
        
        BnQuaternion objectNewQuat = envQuat.mul(rotationRealaxisQuat.mul(envQuat.inverse().mul(startingQuat)));
        
        return objectNewQuat.toList();
    }

}