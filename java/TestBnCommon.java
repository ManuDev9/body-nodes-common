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

import eu.bodynodesdev.common.BnReorientAxis;
import eu.bodynodesdev.common.BnTwoNodesMotionTracking;
import eu.bodynodesdev.common.BnRobotIK_ZYY2Arms;

import java.util.Arrays;

/*
 * javac TestBnCommon.java
 * java TestBnCommon
 *
 * */

public class TestBnCommon {

    private static void testBnReorientAxis() {
        System.out.println("Testing BnReorientAxis");

        int[] test_io_axis = new int[]{ 3, 2, 1, 0 };
        int[] test_io_sign = new int[]{ -1, -1, -1, -1 };
        float[] test_ivalues = new float[]{ 3f, 4.5f, 2f, 10.2f };
        float[] test_ovalues = new float[]{ 3f, 4.5f, 2f, 10.2f };
        float[] test_evalues = new float[]{ -10.2f, -2f, -4.5f, -3f };

        BnReorientAxis test_obj = new BnReorientAxis();
        test_obj.config( test_io_axis, test_io_sign );
        test_obj.apply( test_ovalues );
        if(areArraysClose(test_evalues, test_ovalues, 1e-5f, 1e-3f )) {
            System.out.println("Test passed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        } else {
            System.out.println("Test failed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        }
    }

    private static void testBnTwoNodesMotionTracking() {
        System.out.println("Testing BnTwoNodesMotionTracking");

        float[] test_node1_quat = new float[] { 0.9926f, 0.0329f, 0.0973f, 0.0640f };
        float[] test_node2_quat = new float[] { 0.9583f, -0.1367f, -0.0595f, -0.2439f };
        float[] test_evalues = new float[] { 18.468636171839087f, -3.1761790635934757f, -0.08354223767877755f };

        BnTwoNodesMotionTracking bnmotiontrack = new BnTwoNodesMotionTracking(
            new float[]{ 0,0,0}, 10, 10, new float[][] { { 10, 20 }, { -5, 5}, { -5, 5} }, "cm" );

        float[] test_ovalues = {0, 0, 0};
        bnmotiontrack.compute( test_node1_quat, test_node2_quat, test_ovalues );
        if(areArraysClose(test_evalues, test_ovalues, 1e-2f, 1e-2f )) {
            System.out.println("Test passed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        } else {
            System.out.println("Test failed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        }
    }

    private static void testBnTwoNodesMotionTrackingConstraint() {
        System.out.println("Testing BnTwoNodesMotionTracking Constraint");

        float[] test_node1_quat = new float[] { 0.8504f, 0.3678f, -0.1840f, 0.3281f };
        float[] test_node2_quat = new float[] { 0.9293f, -0.0039f, -0.2892f, 0.2296f };
        float[] test_evalues = new float[] { 14.443218483410508f, 5f, 5f };

        BnTwoNodesMotionTracking bnmotiontrack = new BnTwoNodesMotionTracking(
            new float[]{ 0, 0, 0 }, 10, 10, new float[][] { { 10, 20 }, { -5, 5}, { -5, 5} }, "cm" );

        float[] test_ovalues = { 0, 0, 0 };
        bnmotiontrack.compute( test_node1_quat, test_node2_quat, test_ovalues );
        if(areArraysClose(test_evalues, test_ovalues, 1e-2f, 1e-2f )) {
            System.out.println("Test passed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        } else {
            System.out.println("Test failed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        }
    }

    private static void testBnRobotIK_ZYY2Arms() {
        System.out.println("Testing BnRobotIK_ZYY2Arms");

        float[] test_endpoint = new float[] { 18.219124272891392f, 3.8972461548699857f, 1.6501078154541111f };
        float[] test_evalues = new float[] { 0.21073373345528476f, -0.4522653965641126f, 0.723883473845901f };

        BnRobotIK_ZYY2Arms bnaik = new BnRobotIK_ZYY2Arms(
            10, 10, new float[]{ 0, 0, 0 }, "cm" );

        float[] test_ovalues = { 0, 0, 0 };
        bnaik.compute( test_endpoint, test_ovalues );
        if(areArraysClose(test_evalues, test_ovalues, 1e-5f, 1e-3f )) {
            System.out.println("Test passed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        } else {
            System.out.println("Test failed: output = "+Arrays.toString(test_ovalues) + " expected = "+Arrays.toString(test_evalues));
        }
    }

    /**
     * Checks if two arrays are close to each other within given absolute and relative tolerances.
     * 
     * @param actual Array of actual values.
     * @param expected Array of expected values.
     * @param absError Absolute error tolerance.
     * @param relError Relative error tolerance.
     * @return True if arrays are close within tolerances, false otherwise.
     */
    public static boolean areArraysClose(float[] expected, float[] actual, float absError, float relError) {
        if (actual.length != expected.length) {
            throw new IllegalArgumentException("Arrays must be of the same length.");
        }

        for (int i = 0; i < actual.length; i++) {
            float diff = Math.abs(actual[i] - expected[i]);
            float expectedValue = Math.abs(expected[i]);

            // Check if the difference is within absolute or relative error bounds
            if (diff > absError && (expectedValue == 0 || diff / expectedValue > relError)) {
                return false;
            }
        }

        return true;
    }

    public static void main(String[] args) {
        testBnReorientAxis();
        testBnTwoNodesMotionTracking();
        testBnTwoNodesMotionTrackingConstraint();
        testBnRobotIK_ZYY2Arms();
    }
}
