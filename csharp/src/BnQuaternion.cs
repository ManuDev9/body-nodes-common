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


namespace BodynodesDev.Common
{
    public class BnQuaternion
    {

        private readonly double[] pQvalues;

        public BnQuaternion(double[] vals)
        {
            pQvalues = new double[] { vals[0], vals[1], vals[2], vals[3] };
        }

        public BnQuaternion(double w, double x, double y, double z)
        {
            pQvalues = new double[] { w, x, y, z };
        }

        public BnQuaternion Mul(BnQuaternion otherQ)
        {
            double[] other = otherQ.ToList();

            // Quaternion multiplication
            double w = pQvalues[0] * other[0] - pQvalues[1] * other[1] - pQvalues[2] * other[2] - pQvalues[3] * other[3];
            double x = pQvalues[0] * other[1] + pQvalues[1] * other[0] + pQvalues[2] * other[3] - pQvalues[3] * other[2];
            double y = pQvalues[0] * other[2] + pQvalues[2] * other[0] + pQvalues[3] * other[1] - pQvalues[1] * other[3];
            double z = pQvalues[0] * other[3] + pQvalues[3] * other[0] + pQvalues[1] * other[2] - pQvalues[2] * other[1];

            return new BnQuaternion(w, x, y, z);
        }


        public BnQuaternion Div(double scalar)
        {
            // Division of quaternion by scalar    
            return new BnQuaternion(pQvalues[0] / scalar, pQvalues[1] / scalar, pQvalues[2] / scalar, pQvalues[3] / scalar);
        }

        public BnQuaternion Conjugate()
        {
            // Conjugate of quaternion (inverse for unit quaternions)
            return new BnQuaternion(pQvalues[0], -pQvalues[1], -pQvalues[2], -pQvalues[3]);
        }

        public double Norm()
        {
            // Norm (magnitude) of the quaternion
            return Math.Sqrt(pQvalues[0] * pQvalues[0] + pQvalues[1] * pQvalues[1] + pQvalues[2] * pQvalues[2] + pQvalues[3] * pQvalues[3]);
        }

        public BnQuaternion Inverse()
        {
            // Inverse of the quaternion
            double norm_val = Norm();
            return Conjugate().Div(norm_val * norm_val);
        }

        public override string ToString()
        {
            //  Represent quaternion as a string
            return string.Format("BnQuaternion(w={0:F5}, x={1:F5}, y={2:F5}, z={3:F5})", pQvalues[0], pQvalues[1], pQvalues[2], pQvalues[3]);
        }

        //  Allows accessing the quaternion components using getW(), getX(), getY(), getZ()
        public double GetW()
        {
            return pQvalues[0];
        }
        public double GetX()
        {
            return pQvalues[1];
        }
        public double GetY()
        {
            return pQvalues[2];
        }
        public double GetZ()
        {
            return pQvalues[3];
        }

        public double[] ToList()
        {
            double[] pQvaluesClone = new double[pQvalues.Length];
            Array.Copy(pQvalues, pQvaluesClone, pQvalues.Length);
            return pQvaluesClone;
        }

        public Boolean IsEmpty()
        {
            return pQvalues[0] == 0.0 && pQvalues[1] == 0.0 && pQvalues[2] == 0.0 && pQvalues[3] == 0.0;
        }
    }
}