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

public class BnRobotArm_MT {

    private BnMotionTracking_Interface mMotionTraker;
    private BnRobotIK_Interface mRobotIK;

    public BnRobotArm_MT(BnMotionTracking_Interface motionTraker, BnRobotIK_Interface robotIK){
        mMotionTraker = motionTraker;
        mRobotIK = robotIK;
    }

    // It only needs the angles, the endpoint is computed internally
    // node1_quat and node2_quat are inputs
    // outAngles in an output
    // positions is just to store data, so the MT does not need to do guess what should be the size
    public void compute(double[] node1_quat, double[] node2_quat, double[][] positions, double[][] outAngles) {
        mMotionTraker.compute( node1_quat, node2_quat, positions );
        mRobotIK.compute(positions[positions.length-1], outAngles);
    }

}
