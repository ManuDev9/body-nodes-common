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


class _ConstMeta(type):
    def __setattr__(cls, name, value):
        raise AttributeError("Cannot modify constant values")

    def __delattr__(cls, name):
        raise AttributeError("Cannot delete constant values")


class BnConstants(metaclass=_ConstMeta):

    def __new__(cls):
        raise TypeError("Class cannot be instantiated")

    PLAYER_ALL_TAG = "all"
    PLAYER_NONE_TAG = "none"

    BODYPART_NONE_TAG = "none"
    BODYPART_HEAD_TAG = "head"
    BODYPART_HAND_LEFT_TAG = "hand_left"
    BODYPART_LOWERARM_LEFT_TAG = "lowerarm_left"
    BODYPART_UPPERARM_LEFT_TAG = "upperarm_left"
    BODYPART_BODY_TAG = "body"
    BODYPART_LOWERARM_RIGHT_TAG = "lowerarm_right"
    BODYPART_UPPERARM_RIGHT_TAG = "upperarm_right"
    BODYPART_HAND_RIGHT_TAG = "hand_right"
    BODYPART_LOWERLEG_LEFT_TAG = "lowerleg_left"
    BODYPART_UPPERLEG_LEFT_TAG = "upperleg_left"
    BODYPART_FOOT_LEFT_TAG = "foot_left"
    BODYPART_LOWERLEG_RIGHT_TAG = "lowerleg_right"
    BODYPART_UPPERLEG_RIGHT_TAG = "upperleg_right"
    BODYPART_FOOT_RIGHT_TAG = "foot_right"
    BODYPART_UPPERBODY_TAG = "upperbody"
    BODYPART_LOWERBODY_TAG = "lowerbody"
    BODYPART_KATANA_TAG = "katana"
    BODYPART_UNTAGGED_TAG = "untagged"
    BODYPART_ALL_TAG = "all"

    # ACTIONS
    ACTION_TYPE_TAG = "type"
    ACTION_PLAYER_TAG = "player"
    ACTION_BODYPART_TAG = "bodypart"

    ACTION_TYPE_NONE_TAG = "none"
    ACTION_TYPE_HAPTIC_TAG = "haptic"
    ACTION_TYPE_SETPLAYER_TAG = "set_player"
    ACTION_TYPE_SETBODYPART_TAG = "set_bodypart"
    ACTION_TYPE_ENABLESENSOR_TAG = "enable_sensor"
    ACTION_TYPE_SETWIFI_TAG = "set_wifi"

    ACTION_HAPTIC_DURATION_MS_TAG = "duration_ms"
    ACTION_HAPTIC_STRENGTH_TAG = "strength"
    ACTION_SETPLAYER_NEWPLAYER_TAG = "new_player"
    ACTION_SETBODYPART_NEWBODYPART_TAG = "new_bodypart"
    ACTION_ENABLESENSOR_SENSORTYPE_TAG = "sensortype"
    ACTION_ENABLESENSOR_ENABLE_TAG = "enable"
    ACTION_SETWIFI_SSID_TAG = "ssid"
    ACTION_SETWIFI_PASSWORD_TAG = "password"
    ACTION_SETWIFI_MULTICASTMESSAGE_TAG = "multicast_message"

    # MESSAGE
    MESSAGE_PLAYER_TAG = "player"
    MESSAGE_BODYPART_TAG = "bodypart"
    MESSAGE_SENSORTYPE_TAG = "sensortype"
    MESSAGE_VALUE_TAG = "value"

    # GLOVE
    GLOVE_ANGLE_MIGNOLO_INDEX = 0
    GLOVE_ANGLE_ANULARE_INDEX = 1
    GLOVE_ANGLE_MEDIO_INDEX = 2
    GLOVE_ANGLE_INDICE_INDEX = 3
    GLOVE_ANGLE_POLLICE_INDEX = 4
    GLOVE_TOUCH_MIGNOLO_INDEX = 5
    GLOVE_TOUCH_ANULARE_INDEX = 6
    GLOVE_TOUCH_MEDIO_INDEX = 7
    GLOVE_TOUCH_INDICE_INDEX = 8

    # MEMORY
    MEMORY_BODYPART_TAG = "bodypart"
    MEMORY_BODYPART_GLOVE_TAG = "bodypart_glove"
    MEMORY_BODYPART_SHOE_TAG = "bodypart_shoe"
    MEMORY_PLAYER_TAG = "player"
    MEMORY_WIFI_SSID_TAG = "wifi_ssid"
    MEMORY_WIFI_PASSWORD_TAG = "wifi_password"
    MEMORY_WIFI_MULTICASTMESSAGE_TAG = "multicast_message"

    # SENSOR DATA TYPES
    SENSORTYPE_NONE_TAG = "none"
    SENSORTYPE_ORIENTATION_ABS_TAG = "orientation_abs"
    SENSORTYPE_ACCELERATION_REL_TAG = "acceleration_rel"
    SENSORTYPE_GLOVE_TAG = "glove"
    SENSORTYPE_SHOE_TAG = "shoe"
    SENSORTYPE_ANGULARVELOCITY_REL_TAG = "angularvelocity_rel"

    # SENSOR STATUS
    SENSOR_STATUS_NOT_ACCESSIBLE = 1
    SENSOR_STATUS_CALIBRATING = 2
    SENSOR_STATUS_WORKING = 3

    # CONNECTION STATUS
    CONNECTION_STATUS_NOT_CONNECTED = 0
    CONNECTION_STATUS_WAITING_ACK = 1
    CONNECTION_STATUS_CONNECTED = 2

    # Wifi Connections
    WIFI_PORT = 12345
    WIFI_MULTICAST_PORT = 12346
    WIFI_SSID_DEFAULT = "BodynodeHotspot"
    WIFI_PASSWORD_DEFAULT = "bodynodes1"
    WIFI_MULTICASTGROUP_DEFAULT = "239.192.1.99"
    WIFI_MULTICASTMESSAGE_DEFAULT = "BN"

    # Bluetooth Connections
    # TODO

    # BLE Connections
    BLE_NAME = "Bodynode"
    BLE_SERVICE_UUID = "0000CCA0-0000-1000-8000-00805F9B34FB"
    BLE_CHARA_PLAYER_UUID = "0000CCA1-0000-1000-8000-00805F9B34FB"
    BLE_CHARA_BODYPART_UUID = "0000CCA2-0000-1000-8000-00805F9B34FB"
    BLE_CHARA_ORIENTATION_ABS_VALUE_UUID = "0000CCA3-0000-1000-8000-00805F9B34FB"
    BLE_CHARA_ACCELERATION_REL_VALUE_UUID = "0000CCA4-0000-1000-8000-00805F9B34FB"
    BLE_CHARA_GLOVE_VALUE_UUID = "0000CCA5-0000-1000-8000-00805F9B34FB"
    BLE_CHARA_SHOE_UUID = "0000CCA6-0000-1000-8000-00805F9B34FB"
    BLE_CHARA_ANGULARVELOCITY_REL_VALUE_UUID = "0000CCA7-0000-1000-8000-00805F9B34FB"


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
        return sum([i**2 for i in self.q]) ** 0.5

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


class BnReorientAxis:
    def __init__(self):
        self.reorientAxis = [0, 1, 2, 3]
        self.reorientSign = [1, 1, 1, 1]

    # Change the orientation of received data to align to the main application
    # ioAxis is an array of 3 or 4 integer that can be 0 ('w'), 1 ('x'), 2 ('y'), or 3 ('z').
    # Example [ 0, 1, 2, 3]
    # ioSign is an array of 3 or 4 integers that can be 1 or -1. Example [1, 1, 1, 1]
    # Note: by default the orientation is set as [ 0, 1, 2, 3 ] and [1, 1, 1, 1]
    def config(self, ioAxis, ioSign):
        self.reorientAxis = ioAxis
        self.reorientSign = ioSign

    def apply(self, iovalues):
        ovalues = []
        for idv in range(0, len(iovalues)):
            ovalues.append(iovalues[self.reorientAxis[idv]] * self.reorientSign[idv])
        for idv in range(0, len(iovalues)):
            iovalues[idv] = ovalues[idv]


class BnMotionTracking_2Nodes:

    # This is an example of locationConstraints
    # locationConstraints = [ [-1, 1], [-2, 2], [-3, 3] ]
    def __init__(
        self,
        initialPosition=[0, 0, 0],
        armVector1=[1, 0, 0],
        armVector2=[1, 0, 0],
        locationConstraints=None,
        units="cm",
    ):
        self.initialPosition = initialPosition[:]
        self.armVector1 = armVector1
        self.armVector2 = armVector2
        if locationConstraints != None:
            self.locationConstraints = [item[:] for item in locationConstraints]
        else:
            self.locationConstraints = None
        self.units = units

    def __quaternion_to_rotation_matrix(self, quat):
        return [
            [
                1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]),
                2 * (quat[1] * quat[2] - quat[0] * quat[3]),
                2 * (quat[1] * quat[3] + quat[0] * quat[2]),
            ],
            [
                2 * (quat[1] * quat[2] + quat[0] * quat[3]),
                1 - 2 * (quat[1] * quat[1] + quat[3] * quat[3]),
                2 * (quat[2] * quat[3] - quat[0] * quat[1]),
            ],
            [
                2 * (quat[1] * quat[3] - quat[0] * quat[2]),
                2 * (quat[2] * quat[3] + quat[0] * quat[1]),
                1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]),
            ],
        ]

    def __matrix_multiply_3x3(self, matrix, vector):
        """Multiplies a 3x3 matrix by a 3D vector."""
        result = [0, 0, 0]
        result[0] = (
            matrix[0][0] * vector[0]
            + matrix[0][1] * vector[1]
            + matrix[0][2] * vector[2]
        )
        result[1] = (
            matrix[1][0] * vector[0]
            + matrix[1][1] * vector[1]
            + matrix[1][2] * vector[2]
        )
        result[2] = (
            matrix[2][0] * vector[0]
            + matrix[2][1] * vector[1]
            + matrix[2][2] * vector[2]
        )
        return result

    def compute(self, node1_quat, node2_quat):

        node1_rm = self.__quaternion_to_rotation_matrix(node1_quat)
        node2_rm = self.__quaternion_to_rotation_matrix(node2_quat)

        rotatedArm1 = self.__matrix_multiply_3x3(node1_rm, self.armVector1)
        rotatedArm2 = self.__matrix_multiply_3x3(node2_rm, self.armVector2)

        point1Position = [0, 0, 0]
        point1Position[0] = self.initialPosition[0] + rotatedArm1[0]
        point1Position[1] = self.initialPosition[1] + rotatedArm1[1]
        point1Position[2] = self.initialPosition[2] + rotatedArm1[2]
        point2Position = [0, 0, 0]
        point2Position[0] = point1Position[0] + rotatedArm2[0]
        point2Position[1] = point1Position[1] + rotatedArm2[1]
        point2Position[2] = point1Position[2] + rotatedArm2[2]

        if self.locationConstraints != None:
            if point2Position[0] < self.locationConstraints[0][0]:
                point2Position[0] = self.locationConstraints[0][0]
            elif point2Position[0] > self.locationConstraints[0][1]:
                point2Position[0] = self.locationConstraints[0][1]

            if point2Position[1] < self.locationConstraints[1][0]:
                point2Position[1] = self.locationConstraints[1][0]
            elif point2Position[1] > self.locationConstraints[1][1]:
                point2Position[1] = self.locationConstraints[1][1]

            if point2Position[2] < self.locationConstraints[2][0]:
                point2Position[2] = self.locationConstraints[2][0]
            elif point2Position[2] > self.locationConstraints[2][1]:
                point2Position[2] = self.locationConstraints[2][1]

        return [self.initialPosition, point1Position, point2Position]


class BnRobotIK_ArmZYY:

    # All angles are in radiants
    # This is an example of anglesConstraints
    # anglesConstraints = [ [ -math.pi/2, math.pi/2 ], [0 , math.pi/2], [0, math.pi/2 ] ]
    # Starting Point is assumed to be [0, 0, 0]
    def __init__(
        self, lengthRA1=1, lengthRA2=1, lengthRA3=1, anglesConstraints=None, units="cm"
    ):
        self.lengthRA1 = lengthRA1
        self.lengthRA2 = lengthRA2
        self.lengthRA3 = lengthRA3
        self.anglesConstraints = anglesConstraints
        self.theta_RA1 = None
        self.gamma_RA2 = None
        self.gamma_RA3 = None
        self.units = units

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

        # x rotation alpha
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
        diff_iSP_iEP = [endpoint[0], endpoint[1], endpoint[2] - self.lengthRA1]

        # Length of diff_iSP_iEP
        dist_iSP_iEP_2 = (
            diff_iSP_iEP[0] * diff_iSP_iEP[0]
            + diff_iSP_iEP[1] * diff_iSP_iEP[1]
            + diff_iSP_iEP[2] * diff_iSP_iEP[2]
        )
        dist_iSP_iEP = math.sqrt(dist_iSP_iEP_2)

        # From the low of cosine
        if dist_iSP_iEP == 0:
            gamma_RA2 = 0
        else:
            tmp = (
                self.lengthRA2 * self.lengthRA2
                + dist_iSP_iEP_2
                - self.lengthRA3 * self.lengthRA3
            ) / (2 * self.lengthRA2 * dist_iSP_iEP)
            # Let's try to get the right angle even though the point is unreachable
            tmp = min(max(tmp, -1), 1)
            # TODO evaluate what happens in these scenarios

            # Internal angle start of Arm2 of the triangle Arm2+Arm3+endpoint
            gamma_2_1 = math.acos(tmp)
            # Angle of endpoint and origin Arm2
            if dist_iSP_iEP_2 == 0:
                gamma_2_2 = 0
            else:
                gamma_2_2 = math.asin(diff_iSP_iEP[2] / math.sqrt(dist_iSP_iEP_2))

            # Angle from the global Z axis and local Z axis of Arm2
            gamma_RA2 = math.pi / 2 - (gamma_2_1 + gamma_2_2)

        if self.anglesConstraints != None:
            if gamma_RA2 < self.anglesConstraints[1][0]:
                gamma_RA2 = self.anglesConstraints[1][0]
            elif gamma_RA2 > self.anglesConstraints[1][1]:
                gamma_RA2 = self.anglesConstraints[1][1]

            # Let's recalculate where my intermediate point is in case that the constraints kicked in
            iIP = [
                self.lengthRA2 * math.sin(gamma_RA2) * math.cos(theta_RA1),
                self.lengthRA2 * math.sin(gamma_RA2) * math.sin(theta_RA1),
                self.lengthRA2 * math.cos(gamma_RA2),
            ]
            diff_iIP_iEP = [
                diff_iSP_iEP[0] - iIP[0],
                diff_iSP_iEP[1] - iIP[1],
                diff_iSP_iEP[2] - iIP[2],
            ]
            dist_iIP_iEP_2 = (
                diff_iIP_iEP[0] * diff_iIP_iEP[0]
                + diff_iIP_iEP[1] * diff_iIP_iEP[1]
                + diff_iIP_iEP[2] * diff_iIP_iEP[2]
            )
            dist_iIP_iEP = math.sqrt(dist_iIP_iEP_2)
        else:
            dist_iIP_iEP = self.lengthRA3
            dist_iIP_iEP_2 = dist_iIP_iEP * dist_iIP_iEP

        tmp = (dist_iIP_iEP_2 + self.lengthRA2 * self.lengthRA2 - dist_iSP_iEP_2) / (
            2 * dist_iIP_iEP * self.lengthRA2
        )
        # Let's try to get the right angle even though the point is unreachable
        tmp = min(max(tmp, -1), 1)
        # Internal angle start of Arm3 of the triangle Arm2+Arm3+endpoint, then changed to get the local angle we want
        gamma_RA3 = math.pi - math.acos(tmp)

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

        self.theta_RA1 = theta_RA1
        self.gamma_RA2 = gamma_RA2
        self.gamma_RA3 = gamma_RA3
        return [[0, 0, theta_RA1], [0, gamma_RA2, 0], [0, gamma_RA3, 0]]

    def getEndpoints(self):
        # Get the endpoint of Arm1, Arm2, and Arm3 from last compute() step

        endpointArm1 = [
            0,
            0,
            self.lengthRA1,
        ]

        endpointArm2 = [
            self.lengthRA2 * math.sin(self.gamma_RA2) * math.cos(self.theta_RA1),
            self.lengthRA2 * math.sin(self.gamma_RA2) * math.sin(self.theta_RA1),
            endpointArm1[2] + (self.lengthRA2 * math.cos(self.gamma_RA2)),
        ]

        endpointArm3 = [
            endpointArm2[0]
            + (
                self.lengthRA3
                * math.sin(self.gamma_RA2 + self.gamma_RA3)
                * math.cos(self.theta_RA1)
            ),
            endpointArm2[1]
            + (
                self.lengthRA3
                * math.sin(self.gamma_RA2 + self.gamma_RA3)
                * math.sin(self.theta_RA1)
            ),
            endpointArm2[2]
            + (self.lengthRA3 * math.cos(self.gamma_RA2 + self.gamma_RA3)),
        ]

        return [endpointArm1, endpointArm2, endpointArm3]


# Generic class that can take any IK and MotionTracking algo for a robotic arm
class BnRobotArm_IKMT:

    def __init__(self, motionTraker, robotIK):
        self.motionTraker = motionTraker
        self.robotIK = robotIK

    # It only needs the angles, the endpoint is computed internally
    def compute(self, node1_quat, node2_quat):
        [_, _, endpoint] = self.motionTraker.compute(node1_quat, node2_quat)
        return self.robotIK.compute(endpoint)


####### UTILITIES


class BnUtils:

    def __new__(cls):
        raise TypeError("Class cannot be instantiated")

    # Function to multiply two rotation matrices
    @staticmethod
    def multiply_matrices(R1, R2):
        return np.dot(R1, R2)

    # Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in radians
    @staticmethod
    def blender_euler_to_rotation_matrix_rad(roll, pitch, yaw):
        # counter rh wise rotations
        R_x = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )

        R_y = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        R_z = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        )

        # print("\nResult of Multiplication (R_y * R_x):")
        # print(np.dot(R_y, R_x))

        # Overall rotation matrix (XYZ order)
        R = __class__.multiply_matrices(R_z, __class__.multiply_matrices(R_y, R_x))
        return R

    # Function to compute a rotation matrix from Euler angles (XYZ order). Roll, pitch and yaw are in degrees
    @staticmethod
    def blender_euler_to_rotation_matrix_degree(roll, pitch, yaw):
        return __class__.blender_euler_to_rotation_matrix_rad(
            np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)
        )

    @staticmethod
    def create_quanternion(axis_config, values):
        quat = BnQuaternion(
            [
                axis_config["new_w_sign"] * float(values[axis_config["new_w_val"]]),
                axis_config["new_x_sign"] * float(values[axis_config["new_x_val"]]),
                axis_config["new_y_sign"] * float(values[axis_config["new_y_val"]]),
                axis_config["new_z_sign"] * float(values[axis_config["new_z_val"]]),
            ]
        )
        return quat

    # This trasformation ignores what is the local axis of the object once rotated, we don't trust those axis system which might be compromised
    # sensor_quat               - is the raw quaternion (vector of 4 values) from the sensor
    # first_quat                - is the first quaternion that has been registered from the sensor, this will create a 0 angle from where the sensor started sensing
    # starting_quat             - is the starting quaternion of the object we want to rotated with the sensor
    # env_quat                  - is the environment quaternion, basically indicates somehow where the x axis points
    # bodynodes_axis_config     - axis configuration that will transfor the sensor values into values for the virtual world
    @staticmethod
    def transform_sensor_quat(
        sensor_quat, first_quat, starting_quat, env_quat, bodynodes_axis_config
    ):

        sensor_quat = BnQuaternion(sensor_quat)
        starting_quat = BnQuaternion(starting_quat)
        env_quat = BnQuaternion(env_quat)

        if first_quat == None:
            first_quat = sensor_quat.inverse()
        else:
            first_quat = BnQuaternion(first_quat)

        # bpy.data.objects["katana"].rotation_quaternion = sensor_quat @ first_quat @ starting_quat
        # bpy.data.objects["katana"].rotation_quaternion = starting_quat @ sensor_quat

        # bpy.data.objects["katana"].rotation_quaternion = sensor_quat @ first_quat
        # Definitely wrong, the object rotates differently depeding where the sensor starts

        # bpy.data.objects["katana"].rotation_quaternion = sensor_quat
        # It imposes the current rotation instead of considering the initial rotation of the sensor

        rotation_real_quat = first_quat @ sensor_quat
        rotation_realaxis_quat = __class__.create_quanternion(
            bodynodes_axis_config, rotation_real_quat
        )

        object_new_quat = (
            env_quat @ rotation_realaxis_quat @ env_quat.inverse() @ starting_quat
        )

        return [object_new_quat.to_list(), first_quat.to_list()]


if __name__ == "__main__":
    print("This is a module. Nothing is done just by running it directly")
