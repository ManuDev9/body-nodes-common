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

#include "BnMotionTracking_2Nodes.h"

#include <stddef.h>
#include <string.h>

// double const quat[4], double rotationMatrix[3][3]
static void quaternion_to_rotation_matrix(double const *const quat,
                                          double (*const rotationMatrix)[3]) {
  rotationMatrix[0][0] = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
  rotationMatrix[0][1] = 2 * (quat[1] * quat[2] - quat[0] * quat[3]);
  rotationMatrix[0][2] = 2 * (quat[1] * quat[3] + quat[0] * quat[2]);

  rotationMatrix[1][0] = 2 * (quat[1] * quat[2] + quat[0] * quat[3]);
  rotationMatrix[1][1] = 1 - 2 * (quat[1] * quat[1] + quat[3] * quat[3]);
  rotationMatrix[1][2] = 2 * (quat[2] * quat[3] - quat[0] * quat[1]);

  rotationMatrix[2][0] = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
  rotationMatrix[2][1] = 2 * (quat[2] * quat[3] + quat[0] * quat[1]);
  rotationMatrix[2][2] = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
}

// double const matrix[3][3], double const vector[3], double result[3]
static void matrix_multiply_3x3(double const (*const matrix)[3],
                                double const *const vector,
                                double *const result) {
  result[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] +
              matrix[0][2] * vector[2];
  result[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] +
              matrix[1][2] * vector[2];
  result[2] = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] +
              matrix[2][2] * vector[2];
}

BnMotionTracking_2Nodes_t BnMotionTracking_2Nodes_create(
    double const *const initialPosition, double const *const armVector1,
    double const *const armVector2,
    double const (*const locationConstraints)[2], char const *const units) {

  double tmpLocationConstraints[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  uint8_t tmpHasLocationConstraints = 0;
  if (locationConstraints != NULL) {
    tmpHasLocationConstraints = 1;
    tmpLocationConstraints[0][0] = locationConstraints[0][0];
    tmpLocationConstraints[0][1] = locationConstraints[0][1];
    tmpLocationConstraints[1][0] = locationConstraints[1][0];
    tmpLocationConstraints[1][1] = locationConstraints[1][1];
    tmpLocationConstraints[2][0] = locationConstraints[2][0];
    tmpLocationConstraints[2][1] = locationConstraints[2][1];
  }
  BnMotionTracking_2Nodes_t data = {
      .initialPosition = {initialPosition[0], initialPosition[1],
                          initialPosition[2]},
      .armVector1 = {armVector1[0], armVector1[1], armVector1[2]},
      .armVector2 = {armVector2[0], armVector2[1], armVector2[2]},
      .locationConstraints =
          {{tmpLocationConstraints[0][0], tmpLocationConstraints[0][1]},
           {tmpLocationConstraints[1][0], tmpLocationConstraints[1][1]},
           {tmpLocationConstraints[2][0], tmpLocationConstraints[2][1]}},
      .hasLocationConstraints = tmpHasLocationConstraints};

  strncpy(data.units, units, UNITS_BUFF_SIZE - 1);
  data.units[UNITS_BUFF_SIZE - 1] = '\0'; // safety measure
  return data;
}

void BnMotionTracking_2Nodes_compute(BnMotionTracking_2Nodes_t *const data,
                                     double const *const node1Quat,
                                     double const *const node2Quat,
                                     double (*const endpositions)[3]) {

  double *initialPosition = endpositions[0];
  double *point1Position = endpositions[1];
  double *point2Position = endpositions[2];

  double node1RM[3][3];
  double node2RM[3][3];
  quaternion_to_rotation_matrix(node1Quat, node1RM);
  quaternion_to_rotation_matrix(node2Quat, node2RM);

  double rotatedArm1[3];
  double rotatedArm2[3];

  matrix_multiply_3x3(node1RM, data->armVector1, rotatedArm1);
  matrix_multiply_3x3(node2RM, data->armVector2, rotatedArm2);

  initialPosition[0] = data->initialPosition[0];
  initialPosition[1] = data->initialPosition[1];
  initialPosition[2] = data->initialPosition[2];

  point1Position[0] = initialPosition[0] + rotatedArm1[0];
  point1Position[1] = initialPosition[1] + rotatedArm1[1];
  point1Position[2] = initialPosition[2] + rotatedArm1[2];

  point2Position[0] = point1Position[0] + rotatedArm2[0];
  point2Position[1] = point1Position[1] + rotatedArm2[1];
  point2Position[2] = point1Position[2] + rotatedArm2[2];

  if (data->hasLocationConstraints == 1) {
    if (point2Position[0] < data->locationConstraints[0][0]) {
      point2Position[0] = data->locationConstraints[0][0];
    } else if (point2Position[0] > data->locationConstraints[0][1]) {
      point2Position[0] = data->locationConstraints[0][1];
    }

    if (point2Position[1] < data->locationConstraints[1][0]) {
      point2Position[1] = data->locationConstraints[1][0];
    } else if (point2Position[1] > data->locationConstraints[1][1]) {
      point2Position[1] = data->locationConstraints[1][1];
    }

    if (point2Position[2] < data->locationConstraints[2][0]) {
      point2Position[2] = data->locationConstraints[2][0];
    } else if (point2Position[2] > data->locationConstraints[2][1]) {
      point2Position[2] = data->locationConstraints[2][1];
    }
  }
}
