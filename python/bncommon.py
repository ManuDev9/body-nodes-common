#
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

import math
import numpy as np

class BnReorientAxis:
    def __init__( self ):
        self.reorientAxis = [0, 1, 2, 3]
        self.reorientSign = [1, 1, 1, 1]

    # Change the orientation of received data to align to the main application
    # ioAxis is an array of 3 or 4 integer that can be 0 ('w'), 1 ('x'), 2 ('y'), or 3 ('z').
    # Example [ 0, 1, 2, 3]
    # ioSign is an array of 3 or 4 integers that can be 1 or -1. Example [1, 1, 1, 1]
    # Note: by default the orientation is set as [ 0, 1, 2, 3 ] and [1, 1, 1, 1]
    def config( self, ioAxis, ioSign ):
        self.reorientAxis = ioAxis
        self.reorientSign = ioSign

    def apply( self, iovalues ):
        ovalues = []
        for idv in range(0, len(iovalues)):
            ovalues.append( iovalues[ self.reorientAxis[idv] ] * self.reorientSign[idv]  )
        for idv in range(0, len(iovalues)):
            iovalues[idv] = ovalues[idv]


class BnQuaternion:
    def __init__(self, q):
        if isinstance(q, list) and len(q) == 4:
            # Initialize the quaternion with the list values: [w, x, y, z]
            self.q = q
        else:
            raise ValueError("Input must be a list with 4 elements: [w, x, y, z]")

    def __matmul__(self, other):
        # Quaternion multiplication
        w1, x1, y1, z1 = self.q
        w2, x2, y2, z2 = other.q

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return BnQuaternion([w, x, y, z])

    def __truediv__(self, scalar):
        # Division of quaternion by scalar
        if isinstance(scalar, (int, float)):
            return BnQuaternion([i / scalar for i in self.q])
        return NotImplemented

    def conjugate(self):
        # Conjugate of quaternion (inverse for unit quaternions)
        w, x, y, z = self.q
        return BnQuaternion([w, -x, -y, -z])

    def norm(self):
        # Norm (magnitude) of the quaternion
        return sum([i ** 2 for i in self.q]) ** 0.5

    def inverse(self):
        # Inverse of the quaternion
        return self.conjugate() / (self.norm() ** 2)

    def __repr__(self):
        # Represent quaternion as a string
        return f"Quaternion(w={self.q[0]}, x={self.q[1]}, y={self.q[2]}, z={self.q[3]})"

    def __getitem__(self, idx):
        # Allows accessing the quaternion components using q[idx]
        return self.q[idx]

    def to_list(self):
        # Return quaternion as a simple list
        return self.q


class BnMotionTracking_2Nodes:

    # This is an example of locationConstraints
    # locationConstraints = [ [-1, 1], [-2, 2], [-3, 3] ]
    def __init__( self, initialPosition = [0,0,0], armVector1 = [1, 0, 0], armVector2  = [1, 0, 0], locationConstraints = None, units = "cm"):
        self.initialPosition = initialPosition
        self.armVector1 = armVector1
        self.armVector2 = armVector2
        self.locationConstraints = locationConstraints

    def __quaternion_to_rotation_matrix(self, quat):
        return [
            [1 - 2*(quat[2]*quat[2] + quat[3]*quat[3]), 2*(quat[1]*quat[2] - quat[0]*quat[3]), 2*(quat[1]*quat[3] + quat[0]*quat[2])],
            [2*(quat[1]*quat[2] + quat[0]*quat[3]), 1 - 2*(quat[1]*quat[1] + quat[3]*quat[3]), 2*(quat[2]*quat[3] - quat[0]*quat[1])],
            [2*(quat[1]*quat[3] - quat[0]*quat[2]), 2*(quat[2]*quat[3] + quat[0]*quat[1]), 1 - 2*(quat[1]*quat[1] + quat[2]*quat[2])]
        ]

    def __matrix_multiply_3x3(self, matrix, vector):
        """Multiplies a 3x3 matrix by a 3D vector."""
        result = [0, 0, 0]
        result[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2]
        result[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2]
        result[2] = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2]
        return result
    
    def compute( self, node1_quat, node2_quat ):

        node1_rm = self.__quaternion_to_rotation_matrix(node1_quat)
        node2_rm = self.__quaternion_to_rotation_matrix(node2_quat)
        
        rotatedArm1 = self.__matrix_multiply_3x3(node1_rm, self.armVector1 )
        rotatedArm2 = self.__matrix_multiply_3x3(node2_rm, self.armVector2 )        
        
        finalPosition = [0, 0, 0]
        finalPosition[0] = self.initialPosition[0] + rotatedArm1[0] + rotatedArm2[0]
        finalPosition[1] = self.initialPosition[1] + rotatedArm1[1] + rotatedArm2[1]
        finalPosition[2] = self.initialPosition[2] + rotatedArm1[2] + rotatedArm2[2]

        if self.locationConstraints!= None:
            if finalPosition[0] < self.locationConstraints[0][0]:
                finalPosition[0] = self.locationConstraints[0][0]
            elif finalPosition[0] > self.locationConstraints[0][1]:
                finalPosition[0] = self.locationConstraints[0][1]

            if finalPosition[1] < self.locationConstraints[1][0]:
                finalPosition[1] = self.locationConstraints[1][0]
            elif finalPosition[1] > self.locationConstraints[1][1]:
                finalPosition[1] = self.locationConstraints[1][1]

            if finalPosition[2] < self.locationConstraints[2][0]:
                finalPosition[2] = self.locationConstraints[2][0]
            elif finalPosition[2] > self.locationConstraints[2][1]:
                finalPosition[2] = self.locationConstraints[2][1]

        return finalPosition


class BnRobotArmZYY_IK:

    # All angles are in radiants
    # This is an example of anglesConstraints
    # anglesConstraints = [ [ -math.pi/2, math.pi/2 ], [0 , math.pi/2], [0, math.pi/2 ] ]
    def __init__(self, lengthRA1 = 1, lengthRA2 = 1, lengthRA3 = 1, anglesConstraints = None, units = "cm"):
        self.lengthRA1 = lengthRA1
        self.lengthRA2 = lengthRA2
        self.lengthRA3 = lengthRA3
        self.anglesConstraints = anglesConstraints

    # The returned angles are made to work in blender
    # We are assuming the reset position is all arms along the global Z axis going upwards
    # Lacal x,y,z initially all point in the same way as global x,y,z
    # Output is the local rotation of:
    # - Arm1 around local Z (corresponds to global Z)
    # - Arm2 around local Y
    # - Arm3 around local Y
    # So local X is the pointing rotation
    # Arm1, Arm2, Arm3 are all connected one after the other
    # Arm1 starts at the origin
    # endpoint always refers to the same origin
    def compute(self, endpoint):
    
        # z rotation alpha 
        # y rotation gamma 
        # z rotation theta 
    
        # First let'z find the Z rotation of Arm1 [ x, y, z ]
        if endpoint[0] == 0 and endpoint[1] == 0:
            # Z rotation is not defined if x,y is at the origin
            # we will return Nan
            theta_RA1 = float("nan")
        else:
            theta_RA1 = math.atan2(endpoint[1], endpoint[0])
        
        # Remove Arm1 from the equation, and let's find the endpoint coordinates from Arm2 as origin
        diff_iSP_iEP = [ endpoint[0], endpoint[1], endpoint[2] - self.lengthRA1 ]

        # Length of diff_iSP_iEP
        dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2]
        dist_iSP_iEP = math.sqrt(dist_iSP_iEP_2)

        # From the low of cosine
        if dist_iSP_iEP == 0:
            gamma_RA2 = 0
        else:
            tmp = ( self.lengthRA2*self.lengthRA2 + dist_iSP_iEP_2 - self.lengthRA3*self.lengthRA3 ) / ( 2 * self.lengthRA2 * dist_iSP_iEP )
            # Let's try to get the right angle even though the point is unreachable 
            tmp = min(max( tmp, -1 ), 1)
            #TODO evaluate what happens in these scenarios
            
            # Internal angle start of Arm2 of the triangle Arm2+Arm3+endpoint
            gamma_2_1 = math.acos(tmp)
            # Angle of endpoint and origin Arm2
            if dist_iSP_iEP_2 == 0:
                gamma_2_2 = 0
            else:
                gamma_2_2 = math.asin( diff_iSP_iEP[2] / math.sqrt( dist_iSP_iEP_2 )  )
                
            # Angle from the global Z axis and local Z axis of Arm2
            gamma_RA2 = math.pi/2-(gamma_2_1 + gamma_2_2)

        if self.anglesConstraints != None:
            if gamma_RA2 < self.anglesConstraints[1][0]:
                gamma_RA2 = self.anglesConstraints[1][0]
            elif gamma_RA2 > self.anglesConstraints[1][1]:
                gamma_RA2 = self.anglesConstraints[1][1]

            # Let's recalculate where my intermediate point is in case that the constraints kicked in
            iIP = [ self.lengthRA2*math.sin(gamma_RA2)*math.cos(theta_RA1), self.lengthRA2*math.sin(gamma_RA2)*math.sin(theta_RA1), self.lengthRA2*math.cos(gamma_RA2) ]
            diff_iIP_iEP = [ diff_iSP_iEP[0]-iIP[0], diff_iSP_iEP[1]-iIP[1], diff_iSP_iEP[2]-iIP[2] ]
            dist_iIP_iEP_2 = diff_iIP_iEP[0] * diff_iIP_iEP[0] + diff_iIP_iEP[1] * diff_iIP_iEP[1] + diff_iIP_iEP[2] * diff_iIP_iEP[2]
            dist_iIP_iEP = math.sqrt(dist_iIP_iEP_2)
        else:
            dist_iIP_iEP = self.lengthRA3
            dist_iIP_iEP_2 = dist_iIP_iEP*dist_iIP_iEP

        tmp = ( dist_iIP_iEP_2 + self.lengthRA2*self.lengthRA2 - dist_iSP_iEP_2 ) / ( 2 * dist_iIP_iEP * self.lengthRA2 )
        # Let's try to get the right angle even though the point is unreachable 
        tmp = min(max( tmp, -1 ), 1) 
        # Internal angle start of Arm3 of the triangle Arm2+Arm3+endpoint, then changed to get the local angle we want
        gamma_RA3 = (math.pi - math.acos(tmp))


        if self.anglesConstraints != None:

            if not math.isnan(theta_RA1):
                if theta_RA1 < self.anglesConstraints[0][0]:
                    theta_RA1 = self.anglesConstraints[0][0]
                elif theta_RA1 > self.anglesConstraints[0][1]:
                    theta_RA1 = self.anglesConstraints[0][1]

            if gamma_RA3 < self.anglesConstraints[2][0]:
                gamma_RA3 = self.anglesConstraints[2][0]
            elif gamma_RA3 > self.anglesConstraints[2][1]:
                gamma_RA3 = self.anglesConstraints[2][1]

        return [ theta_RA1, gamma_RA2, gamma_RA3 ]


# Generic class that can take any IK and MotionTracking algo for a robotic arm
class BnRobotArm_MT:

    def __init__(self, motionTraker, robotIK ):
        self.motionTraker = motionTraker
        self.robotIK = robotIK

    # It only needs the angles, the endpoint is computed internally
    def compute(self, node1_quat, node2_quat):
        endpoint = self.motionTraker.compute( node1_quat, node2_quat )
        return self.robotIK.compute(endpoint)


####### UTILITIES


# Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in degrees
def blender_euler_to_rotation_matrix_degree(roll, pitch, yaw):
    return blender_euler_to_rotation_matrix_rad(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))

# Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in radians
def blender_euler_to_rotation_matrix_rad(roll, pitch, yaw):
	# counter rh wise rotations
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    #print("\nResult of Multiplication (R_y * R_x):")
    #print(np.dot(R_y, R_x))

    # Overall rotation matrix (XYZ order)
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

# Function to multiply two rotation matrices
def multiply_rotation_matrices(R1, R2):
    return np.dot(R1, R2)
    

def create_quanternion(axis_config, values):
    
    quat = BnQuaternion([
        axis_config["new_w_sign"] * float(values[axis_config["new_w_val"]]),
        axis_config["new_x_sign"] * float(values[axis_config["new_x_val"]]),
        axis_config["new_y_sign"] * float(values[axis_config["new_y_val"]]),
        axis_config["new_z_sign"] * float(values[axis_config["new_z_val"]])
    ])
    return quat

# This trasformation ignores what is the local axis of the object once rotated, we don't trust those axis system which might be compromised
# sensor_quat               - is the raw quaternion (vector of 4 values) from the sensor
# first_quat                - is the first quaternion that has been registered from the sensor, this will create a 0 angle from where the sensor started sensing
# starting_quat             - is the starting quaternion of the object we want to rotated with the sensor
# env_quat                  - is the environment quaternion, basically indicates somehow where the x axis points 
# bodynodes_axis_config     - axis configuration that will transfor the sensor values into values for the virtual world
def transform_sensor_quat( sensor_quat, first_quat, starting_quat, env_quat, bodynodes_axis_config):

    sensor_quat = BnQuaternion(sensor_quat)
    starting_quat = BnQuaternion(starting_quat)
    env_quat = BnQuaternion(env_quat)

    if first_quat == None:
        first_quat = sensor_quat.inverse()
    else:
        first_quat = BnQuaternion(first_quat)

    #bpy.data.objects["katana"].rotation_quaternion = sensor_quat @ first_quat @ starting_quat 
    #bpy.data.objects["katana"].rotation_quaternion = starting_quat @ sensor_quat
    
    #bpy.data.objects["katana"].rotation_quaternion = sensor_quat @ first_quat 
    # Definitely wrong, the object rotates differently depeding where the sensor starts
    
    #bpy.data.objects["katana"].rotation_quaternion = sensor_quat
    # It imposes the current rotation instead of considering the initial rotation of the sensor

    rotation_real_quat = first_quat @ sensor_quat
    rotation_realaxis_quat = create_quanternion(
            bodynodes_axis_config,
            rotation_real_quat)
    
    object_new_quat = env_quat @ rotation_realaxis_quat @ env_quat.inverse() @ starting_quat
    
    return [ object_new_quat.to_list() , first_quat.to_list() ]


####### TESTS
def test_BnReorientAxis():
    test_res = "Test BnReorientAxis "
    test_io_axis = [ 3, 2, 1, 0 ]
    test_io_sign = [ -1, -1, -1, -1 ]
    test_ivalues = [ 3, 4.5, 2, 10.2 ]
    # ovalues are equal to ivalues for inplace operators
    test_ovalues = [ test_ivalues[0], test_ivalues[1], test_ivalues[2], test_ivalues[3] ]
    test_evalues = [ -10.2, -2, -4.5, -3 ]
    test_obj = BnReorientAxis()
    test_obj.config( test_io_axis, test_io_sign )
    test_obj.apply( test_ovalues )
    
     # Apparently the values come out pretty differently but on the same order of magnitude...
    if np.allclose(test_evalues, test_ovalues, rtol=1e-03, atol=1e-05):
        test_res += "passed"
    else:
        test_res += "failed"
    test_res += " -> \n test_evalues = "+str(test_evalues)+" \n test_ovalues = " +str(test_ovalues)
    print(test_res)

def test_BnMotionTracking_2Nodes():
    test_res = "Test BnMotionTracking_2Nodes "
    bnmotiontrack = BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [10,0,0], armVector2 = [10,0,0],
        locationConstraints = [ [10, 20], [-5, 5], [-5, 5] ])

    test_node1_quat = [ 0.9926, 0.0329, 0.0973, 0.0640 ]
    test_node2_quat = [ 0.9583, -0.1367, -0.0595, -0.2439 ]
    test_evalues = [18.468636171839087, -3.1761790635934757, -0.08354223767877755]
    test_ovalues = bnmotiontrack.compute(test_node1_quat, test_node2_quat)
    if np.allclose(test_evalues, test_ovalues, rtol=1e-02, atol=1e-03):
        test_res += "passed"
    else:
        test_res += "failed"
    test_res += " -> \n test_evalues = "+str(test_evalues)+" \n test_ovalues = " +str(test_ovalues)
    print(test_res)

def test_BnMotionTracking_2Nodes_Constraints():
    test_res = "Test BnMotionTracking_2Nodes Constraints "
    bnmotiontrack = BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [10,0,0], armVector2 = [10,0,0],
        locationConstraints = [ [10, 20], [-5, 5], [-5, 5] ])

    test_node1_quat = [ 0.8504, 0.3678, -0.1840, 0.3281 ]
    test_node2_quat = [ 0.9293, -0.0039, -0.2892, 0.2296 ]
    test_evalues = [14.443218483410508, 5, 5]
    test_ovalues = bnmotiontrack.compute(test_node1_quat, test_node2_quat)
    if np.allclose(test_evalues, test_ovalues, rtol=1e-03, atol=1e-05):
        test_res += "passed"
    else:
        test_res += "failed"
    test_res += " -> \n test_evalues = "+str(test_evalues)+" \n test_ovalues = " +str(test_ovalues)
    print(test_res)


def test_BnRobotArmZYY_IK():
    test_res = "Test BnRobotArmZYY_IK "
    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 10, lengthRA3 = 10,
        units = "cm")

    test_endpoint = [18.219124272891392, 3.8972461548699857, 1.6501078154541111]
    test_evalues = [0.21073373345528476,  1.118530930230784, 0.723883473845901]
    test_ovalues = bnaik.compute(test_endpoint)
    if np.allclose(test_evalues, test_ovalues, rtol=1e-03, atol=1e-05):
        test_res += "passed"
    else:
        test_res += "failed"
    test_res += " -> \n test_evalues = "+str(test_evalues)+" \n test_ovalues = " +str(test_ovalues)
    print(test_res)

def test_BnQuaternion():
    q1 = BnQuaternion([1, 2, 3, 4])  # Quaternion with w=1, x=2, y=3, z=4
    q2 = BnQuaternion([0, 1, 0, 0])  # Another quaternion

    # Multiply quaternions using the @ operator
    q3 = q1 @ q2
    print(f"Result of multiplication: {q3}")

    # Conjugate and inverse
    print(f"Conjugate of q1: {q1.conjugate()}")
    print(f"Inverse of q1: {q1.inverse()}")
    
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
    out = transform_sensor_quat(
        [0.4, 0.3, 0.2, 0.3],
        [0.4, 0.3, 0.2, 0.3],
        [0.4, 0.3, 0.2, 0.3],
        [0.4, 0.3, 0.2, 0.3],
        bodynodes_axis_config)
    print(f"Result of transform_sensor_quat: {out}")


def test_BlenderSimpleLinksProj1():

    # Change the params in the  Blender SimpleLinks project and check that the output point from python is the same as the one shown in Blender
    # bpy.data.objects['CubeI'].matrix_world.translation
    # bpy.data.objects['CubeF'].matrix_world.translation
    # Arm has two links on the y axis

    bnmotiontrack = BnMotionTracking_2Nodes(
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
    print(out)


def test_BlenderSimpleLinksProj2():
    # Testing how to setup the blender utility functions to correspond to what Blender is giving as output. We want rotation XYZ
    out = blender_euler_to_rotation_matrix_degree(45,45,45) @ [0, 1, 0]
    print(out)


def test_BlenderSimpleLinksProj3():
    # Let's check the BnRobotArmZYY_IK
    # The arms length are 0,1,1
    # For this type of test on Blender the Y is the python X axis
    # This is because Blender is forcing me to do this
    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        units = "cm")

    #endpoint = [0, 0, 2] # out should be 0,0,0
    #endpoint = [2, 0, 0] # out should be 0,90,0
    #endpoint = [0, 0, 0] # out should be 0,90,90 -> error
    #endpoint = [1.711, 0.0, 0.703] # out should be 0, 45, 45 
    #endpoint = [1.416, 0.0, 1.416] # out should be 0, 45, 0
    #endpoint = [1.707, 0.0, -0.713] # out should be 0, 90, 45
    #endpoint = [0.713, 0.0, 0.281] # out should be 0, 0, 135
    #endpoint = [0.0, 0.0, 0.0] # out should be nan, 0, 180
    #endpoint = [0.1472482681274414, 0.0, 0.497156023979187] # out should be 0, -60, 150
    #endpoint = [0, 2, 0]# out should be 90,90,0
    #endpoint = [0, -2, 0]# out should be -90,90,0
    #endpoint = [-2, 0, 0]# out should be 180,90,0
    #endpoint = [1.210, 1.210, 0.70] # out should be 45, 45, 45
    endpoint = [1.416094183921814, 1.416094183921814, 0.0]  # out should be 45, 90, 0
    
    out = bnaik.compute(endpoint)
    print(np.rad2deg(out))

def test_BlenderSimpleLinksProj4():
    # Let's test the combination of BnMotionTracking_2Nodes and BnRobotArmZYY_IK
    # So we will decide specific SersorArm1 and SensorArm2 rotations which will move an immaginary point (BnMotionTracking_2Nodes)
    # The RobotArm1, RobotArm2, and RobotArm3 will follow that point (BnRobotArmZYY_IK)
    # The point is valid and no contraints apply at the moment

    # Remember that one makes use of global angles, the second of local angles
    # This is because the sensors have global axis, why motors have local axis
    # Note that you have to probably do some mental gymnastic do match the rotations

    # Arms along the Y axis
    bnmotiontrack = BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [0,1,0], armVector2 = [0,1,0] )
    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        units = "cm")

    # X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    node1_quat = [ 0.92388, 0.382683, 0, 0 ]
    node2_quat = [ 1, 0, 0, 0 ]

    robotMT =  BnRobotArm_MT( bnmotiontrack, bnaik )

    # [90.         45.00005363 44.99993373]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    print(np.rad2deg(robotangles))

    # ---------
    # Arms along the X axis
    bnmotiontrack = BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [1,0,0], armVector2 = [1,0,0] )
    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        units = "cm")

    # Y rotation 80, -20 -> [ 1.12019681930542, 0.0, 0.634331464767456 ]
    node1_quat = [ 0.766044, 0, -0.642788, 0 ]
    node2_quat = [ 0.984808, 0, 0.173648, 0 ]

    robotMT =  BnRobotArm_MT( bnmotiontrack, bnaik )

    # [0.         10. 100.]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    print(np.rad2deg(robotangles))


def test_BlenderSimpleLinksProj5():
    # Let's test the combination of BnMotionTracking_2Nodes and BnRobotArmZYY_IK
    # and some contraints on the Robot.
    # Sensors can give any values but Robots have physical limitions

    # Note that you have to probably do some mental gymnastic do match the rotations
    
    # Arms along the Y axis
    bnmotiontrack = BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [0,1,0], armVector2 = [0,1,0] )
    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -45, 45 ], [0 , 90], [0, 90 ] ]).tolist(),
        units = "cm")

    # X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    node1_quat = [ 0.92388, 0.382683, 0, 0 ]
    node2_quat = [ 1, 0, 0, 0 ]

    robotMT =  BnRobotArm_MT( bnmotiontrack, bnaik )

    # [45.         45.00005363 44.99993373]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    print(np.rad2deg(robotangles))

    # ---------
    # Arms along the Y axis
    bnmotiontrack = BnMotionTracking_2Nodes(
        initialPosition = [0,0,0], armVector1 = [1,0,0], armVector2 = [1,0,0] )
    # The Robot IK will always assume as a starting position the arms to be pointing upwards
    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -90, 90 ], [0 , 90], [0, 90 ] ]).tolist(),
        units = "cm")

    # Z rotation 135
    # Y rotation 10, 20 -> [ -1.3624130487442017, 1.3624131679534912, -0.5175356268882751 ]
    node1_quat = [ 0.381227, -0.080521, 0.033353, 0.920364 ]
    node2_quat = [ 0.37687, -0.16043, 0.066452, 0.909844 ]

    robotMT =  BnRobotArm_MT( bnmotiontrack, bnaik )

    # [90.         90 30]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    print(np.rad2deg(robotangles))

    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -90, 90 ], [0 , 100], [0, 90 ] ]).tolist(),
        units = "cm")
    robotMT =  BnRobotArm_MT( bnmotiontrack, bnaik )
    # [90.         100 10]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    print(np.rad2deg(robotangles))

    bnaik = BnRobotArmZYY_IK(
        lengthRA1 = 0, lengthRA2 = 1, lengthRA3 = 1,
        anglesConstraints = np.deg2rad([ [ -90, 90 ], [0 , 180], [0, 90 ] ]).tolist(),
        units = "cm")
    robotMT =  BnRobotArm_MT( bnmotiontrack, bnaik )
    # [90.         100 10]]
    robotangles = robotMT.compute(node1_quat, node2_quat)
    print(np.rad2deg(robotangles))




if __name__ == "__main__":

    # The X and Y starting position of the Links can create problems in understanding what is the right quanternion and point location
    # To get the right quaternions of Blender feel free to blindly apply the XYZ angles and then get Quaternions
    # Then to get the right point locaiton, just move the angles in a way that will move the point where it should be
    # Then double check the results are the ones you expect. They will be different from what Blender is giving because X and Y are swithed.
    # NOTE: also the rotation positive/negative depends on the right thumb rule. Switched X-Y axis can also mean different rotation signs

    #test_BnReorientAxis()
    #test_BnMotionTracking_2Nodes()
    #test_BnMotionTracking_2Nodes_Constraints()
    #test_BnRobotArmZYY_IK()
    test_BnQuaternion()

    #test_BlenderSimpleLinksProj1()
    #test_BlenderSimpleLinksProj2()
    #test_BlenderSimpleLinksProj3()
    #test_BlenderSimpleLinksProj4()
    #test_BlenderSimpleLinksProj5()


