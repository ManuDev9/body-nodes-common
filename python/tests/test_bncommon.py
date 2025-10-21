#
# MIT License
# 
# Copyright (c) 2025 Manuel Bottini
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

import numpy as np

import bncommon

# The X and Y starting position of the Links can create problems in understanding what is the right quanternion and point location
# To get the right quaternions of Blender feel free to blindly apply the XYZ angles and then get Quaternions
# Then to get the right point locaiton, just move the angles in a way that will move the point where it should be
# Then double check the results are the ones you expect. They will be different from what Blender is giving because X and Y are swithed.
# NOTE: also the rotation positive/negative depends on the right thumb rule. Switched X-Y axis can also mean different rotation signs

def test_BnReorientAxis():
    test_io_axis = [ 3, 2, 1, 0 ]
    test_io_sign = [ -1, -1, -1, -1 ]
    test_ivalues = [ 3, 4.5, 2, 10.2 ]
    # ovalues are equal to ivalues for inplace operators
    test_ovalues = [ test_ivalues[0], test_ivalues[1], test_ivalues[2], test_ivalues[3] ]
    test_evalues = [ -10.2, -2, -4.5, -3 ]
    test_obj = bncommon.BnReorientAxis()
    test_obj.config( test_io_axis, test_io_sign )
    test_obj.apply( test_ovalues )
    
     # Apparently the values come out pretty differently but on the same order of magnitude...
    assert np.allclose(test_evalues, test_ovalues, rtol=1e-03, atol=1e-05)

def test_BnMotionTracking_2Nodes():
    bnmotiontrack = bncommon.BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [10,0,0], armVector2 = [10,0,0],
        locationConstraints = [ [10, 20], [-5, 5], [-5, 5] ])

    test_node1_quat = [ 0.9926, 0.0329, 0.0973, 0.0640 ]
    test_node2_quat = [ 0.9583, -0.1367, -0.0595, -0.2439 ]
    test_evalues = [18.468636171839087, -3.1761790635934757, -0.08354223767877755]
    [ _, _, test_ovalues] = bnmotiontrack.compute(test_node1_quat, test_node2_quat)
    assert np.allclose(test_evalues, test_ovalues, rtol=1e-02, atol=1e-03)

def test_BnMotionTracking_2Nodes_Constraints():
    bnmotiontrack = bncommon.BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [10,0,0], armVector2 = [10,0,0],
        locationConstraints = [ [10, 20], [-5, 5], [-5, 5] ])

    test_node1_quat = [ 0.8504, 0.3678, -0.1840, 0.3281 ]
    test_node2_quat = [ 0.9293, -0.0039, -0.2892, 0.2296 ]
    test_evalues = [14.443218483410508, 5, 5]
    [ _, _, test_ovalues] = bnmotiontrack.compute(test_node1_quat, test_node2_quat)
    assert np.allclose(test_evalues, test_ovalues, rtol=1e-03, atol=1e-05)


def test_BnRobotArmZYY_IK():
    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 10, lengthRA3 = 10,
        units = "cm")

    test_endpoint = [18.219124272891392, 3.8972461548699857, 1.6501078154541111]
    test_evalues = [0.21073373345528476,  1.118530930230784, 0.723883473845901]
    test_ovalues = bnaik.compute(test_endpoint)
    assert np.allclose(test_evalues, test_ovalues, rtol=5e-03, atol=1e-04)

def test_BnQuaternion():
    q1 = bncommon.BnQuaternion([1, 2, 3, 4])  # Quaternion with w=1, x=2, y=3, z=4
    q2 = bncommon.BnQuaternion([0, 1, 0, 0])  # Another quaternion

    # Multiply quaternions using the @ operator
    q3 = q1 @ q2
    assert q3.to_list()  == [-2, 1, 4, -3]

    # Conjugate and inverse
    assert q1.conjugate().to_list() == [1, -2, -3, -4]
    np.allclose(q1.inverse().to_list(), np.array([0.03333333333333333, -0.06666666666666667, -0.1, -0.13333333333333333]) )

    bodynodes_axis_config = {
        "new_w_sign" : 1,
        "new_x_sign" : 1,
        "new_y_sign" : 1,
        "new_z_sign" : 1,
        
        "new_w_val" : 0,
        "new_x_val" : 1,
        "new_y_val" : 2,
        "new_z_val" : 3
    }
    out = bncommon.transform_sensor_quat(
        [0.4, 0.3, 0.2, 0.3],
        [0.4, 0.3, 0.2, 0.3],
        [0.4, 0.3, 0.2, 0.3],
        [0.4, 0.3, 0.2, 0.3],
        bodynodes_axis_config)
    assert out == [[-0.2, 0.07800000000000001, 0.052000000000000005, 0.07800000000000003], [0.4, 0.3, 0.2, 0.3]]


def test_BlenderSimpleLinksProj1():

    # Change the params in the  Blender SimpleLinks project and check that the output point from python is the same as the one shown in Blender
    # bpy.data.objects['CubeI'].matrix_world.translation
    # bpy.data.objects['CubeF'].matrix_world.translation
    # Arm has two links on the y axis

    bnmotiontrack = bncommon.BnMotionTracking_2Nodes(
        initialPosition = [0,0,2], armVector1 = [0,1,0], armVector2 = [0,1,0] )

    # Z rotation 90, -90
    #node1_quat = [ 0.707107, 0, 0, 0.707107 ]
    #node2_quat = [ 0.707107, 0, 0, -0.707107 ]

    # Y rotation 90, -90
    #node1_quat = [ 0.707107, 0, 0.707107, 0 ]
    #node2_quat = [ 0.707107, 0, -0.707107, 0 ]

    # X rotation -90, -90
    node1_quat = [ 0.707107, -0.707107, 0, 0 ]
    node2_quat = [ 0.707107, -0.707107, 0, 0 ]

    out = bnmotiontrack.compute( node1_quat, node2_quat )
    assert out == [[0, 0, 2], [0.0, -6.188980001819999e-07, 0.9999993811019998], [0.0, -1.2377960003639998e-06, -1.2377960003639998e-06]]


def test_BlenderSimpleLinksProj2():
    # Testing how to setup the blender utility functions to correspond to what Blender is giving as output. We want rotation XYZ
    assert np.allclose(bncommon.blender_euler_to_rotation_matrix_degree(45,45,45) @ [0, 1, 0], np.array([-0.14644661,  0.85355339,  0.5  ]) )


def test_BlenderSimpleLinksProj3():
    # Let's check the BnRobotArmZYY_IK
    # The arms length are 0,1,1
    # For this type of test on Blender the Y is the python X axis
    # This is because Blender is forcing me to do this
    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        units = "cm")

    assert np.allclose(np.rad2deg(bnaik.compute([0, 0, 2])), np.array( [np.nan,0,0] ), equal_nan=True )
    assert np.allclose(np.rad2deg(bnaik.compute([2, 0, 0])), np.array( [0,90,0]  ) )
    assert np.allclose(np.rad2deg(bnaik.compute([1.711, 0.0, 0.703])), np.array( [0, 45, 45] ), atol=1 )
    assert np.allclose(np.rad2deg(bnaik.compute([1.416, 0.0, 1.416])), np.array( [0, 45, 0] ), atol=1 )
    assert np.allclose(np.rad2deg(bnaik.compute([1.707, 0.0, -0.713])), np.array( [0, 90, 45] ), atol=1 )
    assert np.allclose(np.rad2deg(bnaik.compute([0.713, 0.0, 0.281])), np.array( [0, 0, 135] ), atol=2 )
    assert np.allclose(np.rad2deg(bnaik.compute([0.0, 0.0, 0.0])), np.array( [np.nan, 0, 180] ), atol=2, equal_nan=True )
    assert np.allclose(np.rad2deg(bnaik.compute([0.1472482681274414, 0.0, 0.497156023979187])), np.array( [0, -60, 150] ) , atol=2)
    assert np.allclose(np.rad2deg(bnaik.compute([0, 2, 0])), np.array( [90,90,0] ) , atol=2)
    assert np.allclose(np.rad2deg(bnaik.compute([0, -2, 0])), np.array( [-90,90,0] ) , atol=2)
    assert np.allclose(np.rad2deg(bnaik.compute([-2, 0, 0])), np.array( [180,90,0] ) , atol=2)
    assert np.allclose(np.rad2deg(bnaik.compute([1.210, 1.210, 0.70])), np.array( [45, 45, 45] ) , atol=2)
    assert np.allclose(np.rad2deg(bnaik.compute([1.416094183921814, 1.416094183921814, 0.0])), np.array( [45, 90,  0] ) , atol=2)

def test_BlenderSimpleLinksProj4():
    # Let's test the combination of BnMotionTracking_2Nodes and BnRobotArmZYY_IK
    # So we will decide specific SersorArm1 and SensorArm2 rotations which will move an immaginary point (BnMotionTracking_2Nodes)
    # The RobotArm1, RobotArm2, and RobotArm3 will follow that point (BnRobotArmZYY_IK)
    # The point is valid and no contraints apply at the moment

    # Remember that one makes use of global angles, the second of local angles
    # This is because the sensors have global axis, why motors have local axis
    # Note that you have to probably do some mental gymnastic do match the rotations

    # Arms along the Y axis
    bnmotiontrack = bncommon.BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [0,1,0], armVector2 = [0,1,0] )
    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        units = "cm")

    # X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    node1_quat = [ 0.92388, 0.382683, 0, 0 ]
    node2_quat = [ 1, 0, 0, 0 ]

    robotMT =  bncommon.BnRobotArm_MT( bnmotiontrack, bnaik )

    # [90.         45.00005363 44.99993373]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    assert np.allclose( np.rad2deg(robotangles), np.array([90, 45.00005363, 44.99993373] ))


    # ---------
    # Arms along the X axis
    bnmotiontrack = bncommon.BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [1,0,0], armVector2 = [1,0,0] )
    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        units = "cm")

    # Y rotation 80, -20 -> [ 1.12019681930542, 0.0, 0.634331464767456 ]
    node1_quat = [ 0.766044, 0, -0.642788, 0 ]
    node2_quat = [ 0.984808, 0, 0.173648, 0 ]

    robotMT =  bncommon.BnRobotArm_MT( bnmotiontrack, bnaik )

    # [0.         10. 100.]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    assert np.allclose( np.rad2deg(robotangles), np.array([0, 9.99994606, 100.00004607] ))


def test_BlenderSimpleLinksProj5():
    # Let's test the combination of BnMotionTracking_2Nodes and BnRobotArmZYY_IK
    # and some contraints on the Robot.
    # Sensors can give any values but Robots have physical limitions

    # Note that you have to probably do some mental gymnastic do match the rotations
    
    # Arms along the Y axis
    bnmotiontrack = bncommon.BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [0,1,0], armVector2 = [0,1,0] )
    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -45, 45 ], [0 , 90], [0, 90 ] ]).tolist(),
        units = "cm")

    # X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    node1_quat = [ 0.92388, 0.382683, 0, 0 ]
    node2_quat = [ 1, 0, 0, 0 ]

    robotMT =  bncommon.BnRobotArm_MT( bnmotiontrack, bnaik )

    # [45.         45.00005363 44.99993373]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    assert np.allclose( np.rad2deg(robotangles), np.array([45, 45.00005363, 44.99993373] ))

    # ---------
    # Arms along the Y axis
    bnmotiontrack = bncommon.BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [1,0,0], armVector2 = [1,0,0] )
    # The Robot IK will always assume as a starting position the arms to be pointing upwards
    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -90, 90 ], [0 , 90], [0, 90 ] ]).tolist(),
        units = "cm")

    # Z rotation 135
    # Y rotation 10, 20 -> [ -1.3624130487442017, 1.3624131679534912, -0.5175356268882751 ]
    node1_quat = [ 0.381227, -0.080521, 0.033353, 0.920364 ]
    node2_quat = [ 0.37687, -0.16043, 0.066452, 0.909844 ]

    robotMT =  bncommon.BnRobotArm_MT( bnmotiontrack, bnaik )

    # [90.         90 30]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    assert np.allclose( np.rad2deg(robotangles), np.array([90, 90, 29.15184909] ))

    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -90, 90 ], [0 , 100], [0, 90 ] ]).tolist(),
        units = "cm")
    robotMT =  bncommon.BnRobotArm_MT( bnmotiontrack, bnaik )
    # [90.         100 10]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    assert np.allclose( np.rad2deg(robotangles), np.array([90, 100, 9.9999254] ))

    bnaik = bncommon.BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -90, 90 ], [0 , 180], [0, 90 ] ]).tolist(),
        units = "cm")
    robotMT =  bncommon.BnRobotArm_MT( bnmotiontrack, bnaik )
    # [90.         100 10]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    assert np.allclose( np.rad2deg(robotangles), np.array([90, 100.00035347, 9.99922384] ))
