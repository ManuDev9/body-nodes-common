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

public class BnQuaternion {

  private final double[] pQvalues;

  public BnQuaternion(double[] vals) {
    pQvalues = new double[] {vals[0], vals[1], vals[2], vals[3]};
  }

  public BnQuaternion(double w, double x, double y, double z) {
    pQvalues = new double[] {w, x, y, z};
  }

  public BnQuaternion mul(BnQuaternion otherQ) {
    final double[] other = otherQ.toList();

    // Quaternion multiplication
    double w =
        pQvalues[0] * other[0]
            - pQvalues[1] * other[1]
            - pQvalues[2] * other[2]
            - pQvalues[3] * other[3];
    double x =
        pQvalues[0] * other[1]
            + pQvalues[1] * other[0]
            + pQvalues[2] * other[3]
            - pQvalues[3] * other[2];
    double y =
        pQvalues[0] * other[2]
            + pQvalues[2] * other[0]
            + pQvalues[3] * other[1]
            - pQvalues[1] * other[3];
    double z =
        pQvalues[0] * other[3]
            + pQvalues[3] * other[0]
            + pQvalues[1] * other[2]
            - pQvalues[2] * other[1];

    return new BnQuaternion(w, x, y, z);
  }

  public BnQuaternion div(double scalar) {
    // Division of quaternion by scalar
    return new BnQuaternion(
        pQvalues[0] / scalar, pQvalues[1] / scalar, pQvalues[2] / scalar, pQvalues[3] / scalar);
  }

  public BnQuaternion conjugate() {
    // Conjugate of quaternion (inverse for unit quaternions)
    return new BnQuaternion(pQvalues[0], -pQvalues[1], -pQvalues[2], -pQvalues[3]);
  }

  public double norm() {
    // Norm (magnitude) of the quaternion
    return Math.sqrt(
        pQvalues[0] * pQvalues[0]
            + pQvalues[1] * pQvalues[1]
            + pQvalues[2] * pQvalues[2]
            + pQvalues[3] * pQvalues[3]);
  }

  public BnQuaternion inverse() {
    // Inverse of the quaternion
    final double norm_val = norm();
    return conjugate().div(norm_val * norm_val);
  }

  @Override
  public String toString() {
    //  Represent quaternion as a string
    return String.format(
        "BnQuaternion(w=%.5f, x=%.5f, y=%.5f, z=%.5f)",
        pQvalues[0], pQvalues[1], pQvalues[2], pQvalues[3]);
  }

  //  Allows accessing the quaternion components using getW(), getX(), getY(), getZ()
  public double getW() {
    return pQvalues[0];
  }

  public double getX() {
    return pQvalues[1];
  }

  public double getY() {
    return pQvalues[2];
  }

  public double getZ() {
    return pQvalues[3];
  }

  public double[] toList() {
    return pQvalues.clone();
  }

  public Boolean isEmpty() {
    return pQvalues[0] == 0.0 && pQvalues[1] == 0.0 && pQvalues[2] == 0.0 && pQvalues[3] == 0.0;
  }
}
