#
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

class BnTwoNodesMotionTracking:

    # Both arms are considered to align on the X axis, with a rotation of 0,0,0
    # This means that lengthArm1 and lengthArm2 are going to influence only the X position of the points
    # This is an example of locationConstraints
    # locationConstraints = [ [-1, 1], [-2, 2], [-3, 3] ]
    def __init__( self, initialPosition = [0,0,0], lengthArm1 = 1, lengthArm2 = 1, locationConstraints = None, units = "cm"):
        self.initialPosition = initialPosition
        self.lengthArm1 = lengthArm1
        self.lengthArm2 = lengthArm2
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

        rotatedArm1 = self.__matrix_multiply_3x3(node1_rm, [ self.lengthArm1, 0, 0 ] )
        rotatedArm2 = self.__matrix_multiply_3x3(node2_rm, [ self.lengthArm2, 0, 0 ] )
        
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


class BnRobotIK_ZYY2Arms:

    # Starting Point is assumed to be [0, 0, 0]
    def __init__(self, lengthRA2 = 1, lengthRA3 = 1, displSP = [0, 0, 0], units = "cm"):
        self.lengthRA2 = lengthRA2
        self.lengthRA3 = lengthRA3
        self.displSP = displSP
        
    # The returned angles are made to work in blender
    def compute(self, endpoint):
    
        theta_RA1 = math.asin( ( endpoint[1] ) / math.sqrt( endpoint[0]*endpoint[0] + endpoint[1]*endpoint[1] ) )
        
        diff_iSP_iEP = [ endpoint[0] - self.displSP[0], endpoint[1] - self.displSP[1], endpoint[2] - self.displSP[2] ]
        
        dist_iSP_iEP_2 = diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[1] * diff_iSP_iEP[1] + diff_iSP_iEP[2] * diff_iSP_iEP[2]
        dist_iSP_iEP = math.sqrt(dist_iSP_iEP_2)
        tmp = ( self.lengthRA2*self.lengthRA2 + dist_iSP_iEP_2 - self.lengthRA3*self.lengthRA3 ) / ( 2 * self.lengthRA2 * dist_iSP_iEP )
        
        gamma_2_1 = math.acos(tmp)
        gamma_2_2 = math.asin( diff_iSP_iEP[2] / math.sqrt( diff_iSP_iEP[0] * diff_iSP_iEP[0] + diff_iSP_iEP[2] * diff_iSP_iEP[2] )  )
        gamma_RA2 = -(gamma_2_1 + gamma_2_2)

        tmp = ( self.lengthRA3*self.lengthRA3 + self.lengthRA2*self.lengthRA2 - dist_iSP_iEP_2 ) / ( 2 * self.lengthRA3 * self.lengthRA2 )
        gamma_RA3 = (math.pi - math.acos(tmp))

        return [ theta_RA1, gamma_RA2, gamma_RA3 ]

if __name__ == "__main__":
    reltol = 1e-03
    abstol = 1e-05

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
    if np.allclose(test_evalues, test_ovalues, rtol=reltol, atol=abstol):
        test_res += "passed"
    else:
        test_res += "failed"
    test_res += " -> \n test_evalues = "+str(test_evalues)+" \n test_ovalues = " +str(test_ovalues)
    print(test_res)

    test_res = "Test BnTwoNodesMotionTracking "
    bnmotiontrack = BnTwoNodesMotionTracking(
        initialPosition = [0,0,0], lengthArm1 = 10, lengthArm2 = 10,
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

    test_res = "Test BnTwoNodesMotionTracking Constraints "
    bnmotiontrack = BnTwoNodesMotionTracking(
        initialPosition = [0,0,0], lengthArm1 = 10, lengthArm2 = 10,
        locationConstraints = [ [10, 20], [-5, 5], [-5, 5] ])

    test_node1_quat = [ 0.8504, 0.3678, -0.1840, 0.3281 ]
    test_node2_quat = [ 0.9293, -0.0039, -0.2892, 0.2296 ]
    test_evalues = [14.443218483410508, 5, 5]
    test_ovalues = bnmotiontrack.compute(test_node1_quat, test_node2_quat)
    if np.allclose(test_evalues, test_ovalues, rtol=reltol, atol=abstol):
        test_res += "passed"
    else:
        test_res += "failed"
    test_res += " -> \n test_evalues = "+str(test_evalues)+" \n test_ovalues = " +str(test_ovalues)
    print(test_res)

    test_res = "Test BnRobotIK_ZYY2Arms "
    bnaik = BnRobotIK_ZYY2Arms(
        lengthRA2 = 10, lengthRA3 = 10,
        displSP = [0, 0, 0],
        units = "cm")

    test_endpoint = [18.219124272891392, 3.8972461548699857, 1.6501078154541111]
    test_evalues = [0.21073373345528476, -0.4522653965641126, 0.723883473845901]
    test_ovalues = bnaik.compute(test_endpoint)
    if np.allclose(test_evalues, test_ovalues, rtol=reltol, atol=abstol):
        test_res += "passed"
    else:
        test_res += "failed"
    test_res += " -> \n test_evalues = "+str(test_evalues)+" \n test_ovalues = " +str(test_ovalues)
    print(test_res)




