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
namespace csharp.Tests;

using BodynodesDev.Common;
using NUnit.Framework;
using NUnit.Framework.Constraints;

using System.Reflection;
using Newtonsoft.Json;

/*
 * dotnet new console
 *
 *
 * dotnet test
 * dotnet run
 */

[TestFixture]
public class TestBnCommon
{

    public static bool ArraysAreClose(int[] actual, int[] expected, int absError, double relError)
    {
        if (actual.Length != expected.Length)
        {
            throw new Exception("Arrays must be of the same length.");
        }

        for (int i = 0; i < actual.Length; i++)
        {
            int diff = Math.Abs(actual[i] - expected[i]);
            int expectedValue = Math.Abs(expected[i]);

            // Check if the difference is within absolute or relative error bounds
            if (diff > absError && (expectedValue == 0 || (double)(diff) / (double)(expectedValue) > relError))
            {
                return false;
            }
        }

        return true;
    }

    public static bool ArraysAreClose(double[] actual, double[] expected, double absError, double relError)
    {
        if (actual.Length != expected.Length)
        {
            throw new Exception("Arrays must be of the same length.");
        }

        for (int i = 0; i < actual.Length; i++)
        {
            double diff = Math.Abs(actual[i] - expected[i]);
            double expectedValue = Math.Abs(expected[i]);

            // Check if the difference is within absolute or relative error bounds
            if (diff > absError && (expectedValue == 0 || diff / expectedValue > relError))
            {
                return false;
            }
        }

        return true;
    }


    public static bool ArraysAreClose(float[] actual, float[] expected, float absError, float relError)
    {
        if (actual.Length != expected.Length)
        {
            throw new Exception("Arrays must be of the same length.");
        }

        for (int i = 0; i < actual.Length; i++)
        {
            float diff = Math.Abs(actual[i] - expected[i]);
            float expectedValue = Math.Abs(expected[i]);

            // Check if the difference is within absolute or relative error bounds
            if (diff > absError && (expectedValue == 0 || diff / expectedValue > relError))
            {
                return false;
            }
        }

        return true;
    }


    [Test]
    public void Test_BnConstants()
    {
        // Test private constructor
        var constructor = typeof(BnConstants).GetConstructor(
            BindingFlags.NonPublic | BindingFlags.Instance,
            null, Type.EmptyTypes, null);

        Assert.That(constructor != null, "Cannot access private constructor via reflection");

        bool constructorThrewException = false;
        try
        {
            constructor!.Invoke(null);
        }
        catch (TargetInvocationException ex) when (ex.InnerException is NotSupportedException)
        {
            constructorThrewException = true;
        }
        catch (Exception ex) when (ex.InnerException?.GetType().Name == "NotSupportedException")
        {
            constructorThrewException = true;
        }
        Assert.That(constructorThrewException, "Expected NotSupportedException from constructor");

        // Test field immutability
        var field = typeof(BnConstants).GetField("ACTION_BODYPART_TAG",
            BindingFlags.Public | BindingFlags.Static);

        Assert.That(field != null, "Expected field not found: ACTION_BODYPART_TAG");

        bool fieldSetThrewException = false;
        try
        {
            field!.SetValue(null, "new value");
        }
        catch (FieldAccessException)
        {
            fieldSetThrewException = true;
        }
        catch (NotSupportedException)
        {
            fieldSetThrewException = true;
        }
        Assert.That(fieldSetThrewException, "Expected FieldAccessException or NotSupportedException when setting field");

        // Load JSON file (relative to project root)
        string jsonPath = Path.GetFullPath(Path.Combine("..", "..", "..", "..", "..", "..", "BNCONSTANTS.json"));
        string jsonContent = File.ReadAllText(jsonPath);

        var allConstants = JsonConvert.DeserializeObject<Dictionary<string, object>>(jsonContent);

        // Remove keys starting with "__"
        var filteredConstants = allConstants!
            .Where(kvp => !kvp.Key.StartsWith("__"))
            .ToDictionary(kvp => kvp.Key, kvp => kvp.Value);

        // Check all constants exist in BnConstants
        var missing = new List<string>();
        foreach (string key in filteredConstants.Keys)
        {
            var constantField = typeof(BnConstants).GetField(key,
                BindingFlags.Public | BindingFlags.Static);
            if (constantField == null)
            {
                missing.Add(key);
            }
        }

        Assert.That(!missing.Any(), $"Missing constants: {string.Join(", ", missing)}");

        // Check values match
        foreach (var entry in filteredConstants)
        {
            string key = entry.Key;
            object expected = entry.Value;

            try
            {
                var constantField = typeof(BnConstants).GetField(key,
                    BindingFlags.Public | BindingFlags.Static);
                object? actual = constantField!.GetValue(null);

                //Console.WriteLine($"Expected type: {expected.GetType()}, Value: {expected}");
                //Console.WriteLine($"Actual type: {actual.GetType()}, Value: {actual}");
                bool valuesMatch = false;
                if (expected.GetType() == typeof(long))
                {
                    uint expectedUint = Convert.ToUInt32(expected);
                    uint actualUint = Convert.ToUInt32(actual);
                    valuesMatch = expectedUint == actualUint;
                }
                else
                {
                    valuesMatch = expected.Equals(actual);
                }

                Assert.That(valuesMatch, $"Mismatch for constant: {key}. Expected: {expected}, Actual: {actual}");
            }
            catch (Exception ex)
            {
                Assert.That(false, $"Error reading field '{key}': {ex.Message}");
            }
        }
    }


    [Test]
    public void Test_BnQuaternion()
    {

        BnQuaternion q1 = new BnQuaternion(1, 2, 3, 4);
        BnQuaternion q2 = new BnQuaternion(0, 1, 0, 0);

        BnQuaternion q3 = q1.Mul(q2);
        Assert.That(ArraysAreClose(q3.ToList(), new double[] { -2, 1, 4, -3 }, 1e-9, 1e-6f));

        // Conjugate and inverse
        Assert.That(ArraysAreClose(q1.Conjugate().ToList(), new double[] { 1, -2, -3, -4 }, 1e-9, 1e-6f));
        Assert.That(ArraysAreClose(q1.Inverse().ToList(), new double[] { 0.03333333333333333, -0.06666666666666667, -0.1, -0.13333333333333333 }, 1e-9, 1e-6f));

    }


    [Test]
    public void Test_BnUtils()
    {

        double[][] actual = BnUtils.blender_euler_to_rotation_matrix_rad(0.123, 2.12, -3.11);
        double[][] expected = new double[][] {
            new double[] { 0.52174769, -0.07324637, -0.8499496 },
            new double[] { 0.01648888, -0.99525533,  0.09589024},
            new double[] {-0.85294048, -0.06404523, -0.51806442}};
        for (int i = 0; i < expected.Length; i++)
        {
            Assert.That(ArraysAreClose(expected[i], actual[i], 0.001, 1e-2f));
        }

        actual = BnUtils.blender_euler_to_rotation_matrix_degree(30, 40, 50);
        expected = new double[][] {
            new double[] { 0.49240388, -0.45682599,  0.74084306 },
            new double[] {0.58682409,  0.80287234,  0.10504046},
            new double[] {-0.64278761,  0.38302222,  0.66341395}};
        for (int i = 0; i < expected.Length; i++)
        {
            Assert.That(ArraysAreClose(expected[i], actual[i], 0.001, 1e-2f));
        }

        actual = BnUtils.MultiplyRotationMatrices(
            new double[][] {
                new double[] {1,2,3},
                new double[] {4,5,6},
                new double[] {1,2,3}},
            new double[][] {
                new double[] {4,5,3},
                new double[] {1,5,8},
                new double[] {9,1,4}});
        expected = new double[][] {
            new double[] {33, 18, 31},
            new double[] {75, 51, 76},
            new double[] {33, 18, 31}};

        for (int i = 0; i < expected.Length; i++)
        {
            Assert.That(ArraysAreClose(expected[i], actual[i], 0.001, 1e-2f));
        }

        BnAxisConfig axisConfig = new BnAxisConfig(1, -1, -1, 1, 0, 1, 3, 2);
        Assert.That(ArraysAreClose(BnUtils.CreateQuanternion(axisConfig, new double[] { 3, 4, 2, 1 }).ToList(), new BnQuaternion(3.0, -4.0, -1.0, 2.0).ToList(), 1e-9, 1e-6f));

        double[] firstQuatVals = new double[] { 0, 0, 0, 0 };
        Assert.That(
            ArraysAreClose(
                BnUtils.TransformSensorQuat(
                    new double[] { 0.4, 0.3, 0.2, 0.3 },
                    firstQuatVals,
                    new double[] { 0.4, 0.3, 0.2, 0.3 },
                    new double[] { 0.4, 0.3, 0.2, 0.3 },
                    new int[] { 1, -1, -1, 1, 0, 1, 3, 2 }),
                new double[] { 0.39999999999999997, 0.2999999999999999, 0.19999999999999998, 0.29999999999999993 },
                1e-9, 1e-6f));

        Assert.That(ArraysAreClose(firstQuatVals, new double[] { 1.0526315789473684, -0.7894736842105263, -0.5263157894736842, -0.7894736842105263 }, 1e-9, 1e-6f));

        firstQuatVals = new double[] { 0.4, 0.3, 0.2, 0.3 };
        Assert.That(
            ArraysAreClose(
                BnUtils.TransformSensorQuat(
                    new double[] { 0.4, 0.3, 0.2, 0.3 },
                    firstQuatVals,
                    new double[] { 0.4, 0.3, 0.2, 0.3 },
                    new double[] { 0.4, 0.3, 0.2, 0.3 },
                    new int[] { 1, -1, -1, 1, 0, 1, 3, 2 }),
                new double[] { 0.048, -0.009999999999999974, -0.22799999999999995, 0.022000000000000006 },
                1e-9, 1e-6f));

        Assert.That(ArraysAreClose(firstQuatVals, new double[] { 0.4, 0.3, 0.2, 0.3 }, 1e-9, 1e-6f));
    }


    [Test]
    public void Test_BnAxisConfig()
    {

        int[] test_io_axis = new int[] { 3, 2, 1, 0 };
        int[] test_io_sign = new int[] { -1, -1, -1, -1 };
        BnAxisConfig test_obj = new BnAxisConfig();
        test_obj.Config(test_io_axis, test_io_sign);

        float[] test_ivaluesF = new float[] { 3f, 4.5f, 2f, 10.2f };
        //  ovalues are equal to ivalues for inplace operators
        float[] test_ovaluesF = new float[] { test_ivaluesF[0], test_ivaluesF[1], test_ivaluesF[2], test_ivaluesF[3] };
        float[] test_evaluesF = new float[] { -10.2f, -2f, -4.5f, -3f };
        test_obj.Apply(test_ovaluesF);
        Assert.That(ArraysAreClose(test_ovaluesF, test_evaluesF, 1e-9f, 1e-6f));

        double[] test_ivaluesD = new double[] { 3, 4.5, 2, 10.2 };
        //  ovalues are equal to ivalues for inplace operators
        double[] test_ovaluesD = new double[] { test_ivaluesD[0], test_ivaluesD[1], test_ivaluesD[2], test_ivaluesD[3] };
        double[] test_evaluesD = new double[] { -10.2, -2, -4.5, -3 };
        test_obj.Apply(test_ovaluesD);
        Assert.That(ArraysAreClose(test_ovaluesD, test_evaluesD, 1e-9f, 1e-6f));

        int[] test_ivaluesI = new int[] { 3, 4, 2, 10 };
        //  ovalues are equal to ivalues for inplace operators
        int[] test_ovaluesI = new int[] { test_ivaluesI[0], test_ivaluesI[1], test_ivaluesI[2], test_ivaluesI[3] };
        int[] test_evaluesI = new int[] { -10, -2, -4, -3 };
        test_obj.Apply(test_ovaluesI);
        Assert.That(ArraysAreClose(test_ovaluesI, test_evaluesI, 0, 1e-6f));
    }

    [Test]
    public void Test_BnMotionTracking_2Nodes()
    {

        BnMotionTracking_2Nodes bnmotiontrack = new BnMotionTracking_2Nodes(
            new double[] { 0, 0, 0 }, new double[] { 10, 0, 0 }, new double[] { 10, 0, 0 }, null, "cm");

        double[] test_node1_quat = new double[] { 0.9926, 0.0329, 0.0973, 0.0640 };
        double[] test_node2_quat = new double[] { 0.9583, -0.1367, -0.0595, -0.2439 };
        double[] test_evalues = new double[] { 18.468636171839087, -3.1761790635934757, -0.08354223767877755 };

        double[][] test_ovalues = new double[][] {
            new double[] { 1, 1, 1 },
            new double[] { 2, 2, 2 },
            new double[] { 3, 3, 3 }};
        bnmotiontrack.Compute(test_node1_quat, test_node2_quat, test_ovalues);
        Assert.That(ArraysAreClose(test_ovalues[2], test_evalues, 1e-2f, 1e-2f));
    }

    [Test]
    public void Test_BnMotionTracking_2NodesConstraint()
    {

        BnMotionTracking_2Nodes bnmotiontrack = new BnMotionTracking_2Nodes(
            new double[] { 0, 0, 0 }, new double[] { 10, 0, 0 }, new double[] { 10, 0, 0 },
            new double[][] {
                new double[] { 10, 20 },
                new double[] { -5, 5},
                new double[] { -5, 5} }, "cm");

        double[] test_node1_quat = new double[] { 0.8504, 0.3678, -0.1840, 0.3281 };
        double[] test_node2_quat = new double[] { 0.9293, -0.0039, -0.2892, 0.2296 };
        double[] test_evalues = new double[] { 14.443218483410508, 5, 5 };

        double[][] test_ovalues = new double[][] {
            new double[] { 1, 1, 1 },
            new double[] { 2, 2, 2 },
            new double[] { 3, 3, 3 }};
        bnmotiontrack.Compute(test_node1_quat, test_node2_quat, test_ovalues);
        Assert.That(ArraysAreClose(test_ovalues[2], test_evalues, 1e-2f, 1e-2f));
    }


    [Test]
    public void Test_BnRobotIK_ArmZYY()
    {

        double[] test_endpoint = new double[] { 18.219124272891392, 3.8972461548699857, 1.6501078154541111 };
        BnRobotIK_ArmZYY bnaik = new BnRobotIK_ArmZYY(0, 10, 10, null, "cm");

        double[][] test_evalues = new double[][] {
            new double[] { 0, 0, 0.21073373345528476 },
            new double[] { 0, 1.120530930230784, 0 },
            new double[] { 0, 0.723883473845901, 0 }};
        double[][] test_ovalues = new double[][] {
            new double[] { 1, 1, 1 },
            new double[] { 2, 2, 2 },
            new double[] { 3, 3, 3 }};
        bnaik.Compute(test_endpoint, test_ovalues);
        Assert.That(ArraysAreClose(test_ovalues[0], test_evalues[0], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_ovalues[1], test_evalues[1], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_ovalues[2], test_evalues[2], 1e-2f, 1e-2f));
    }


    [Test]
    public void Test_BlenderSimpleLinksProj1()
    {

        BnMotionTracking_2Nodes bnmotiontrack = new BnMotionTracking_2Nodes(
            new double[] { 0, 0, 2 }, new double[] { 0, 1, 0 }, new double[] { 0, 1, 0 }, null, "cm");

        // X rotation -90, -90
        double[] node1_quat = new double[] { 0.707107, -0.707107, 0, 0 };
        double[] node2_quat = new double[] { 0.707107, -0.707107, 0, 0 };

        double[][] test_evalues = new double[][] {
            new double[] {0, 0, 2},
            new double[] {0.0, -6.188980001819999e-07, 0.9999993811019998},
            new double[] {0.0, -1.2377960003639998e-06, -1.2377960003639998e-06},
        };

        double[][] test_ovalues = new double[][] {
            new double[] {0, 0, 0},
            new double[] {0, 0, 0},
            new double[] {0, 0, 0},
        };
        bnmotiontrack.Compute(node1_quat, node2_quat, test_ovalues);
        Assert.That(ArraysAreClose(test_ovalues[0], test_evalues[0], 1e-3f, 1e-3f));
        Assert.That(ArraysAreClose(test_ovalues[1], test_evalues[1], 1e-3f, 1e-3f));
        Assert.That(ArraysAreClose(test_ovalues[2], test_evalues[2], 1e-3f, 1e-3f));
    }

    [Test]
    public void Test_BlenderSimpleLinksProj2()
    {
        // Testing how to setup the blender utility functions to correspond to what Blender is giving as output. We want rotation XYZ
        double[][] rot1 = BnUtils.blender_euler_to_rotation_matrix_degree(45, 45, 45);
        double[] vec1 = new double[] { 0, 1, 0 };
        double[] test_evalues = new double[] { -0.14644661, 0.85355339, 0.5 };
        Assert.That(ArraysAreClose(test_evalues, BnUtils.MultiplyRotationMatrixWithVector(rot1, vec1), 1e-6f, 1e-5f));
    }


    [Test]
    public void Test_BlenderSimpleLinksProj3()
    {
        // Let's check the BnRobotIK_ArmZYY
        // The arms length are 0,1,1
        // For this type of test on Blender the Y is the python X axis
        // This is because Blender is forcing me to do this
        BnRobotIK_ArmZYY bnaik = new BnRobotIK_ArmZYY(
            0, 1, 1, null, "cm");

        double[][] test_evalues = new double[][] {
            new double[] { 0, 0, Double.NaN },
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 }};
        double[][] test_ovalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 }};
        bnaik.Compute(new double[] { 0, 0, 2 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        test_evalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(90), 0 },
            new double[] { 0, 0, 0 }};
        bnaik.Compute(new double[] { 2, 0, 0 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        test_evalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(45), 0 },
            new double[] { 0, BnUtils.ToRadians(45), 0 }};
        bnaik.Compute(new double[] { 1.711, 0.0, 0.703 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-2f, 1e-2f));

        test_evalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(45), 0 },
            new double[] { 0, 0, 0 }};
        bnaik.Compute(new double[] { 1.416, 0.0, 1.416 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        test_evalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(90), 0 },
            new double[] { 0, BnUtils.ToRadians(45), 0 }};
        bnaik.Compute(new double[] { 1.707, 0.0, -0.713 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-2f, 1e-2f));

        test_evalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(135), 0 }};
        bnaik.Compute(new double[] { 0.713, 0.0, 0.281 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-1f, 1e-1f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-1f, 1e-1f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-1f, 1e-1f));

        test_evalues = new double[][] {
            new double[] { 0, 0, Double.NaN },
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(180), 0 }};
        bnaik.Compute(new double[] { 0.0, 0.0, 0.0 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        test_evalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(-60), 0 },
            new double[] { 0, BnUtils.ToRadians(150), 0 }};
        bnaik.Compute(new double[] { 0.1472482681274414, 0.0, 0.497156023979187 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-1f, 1e-1f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-1f, 1e-1f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-1f, 1e-1f));

        test_evalues = new double[][] {
            new double[] { 0, 0, BnUtils.ToRadians(90) },
            new double[] { 0, BnUtils.ToRadians(90), 0 },
            new double[] { 0, 0, 0 }};
        bnaik.Compute(new double[] { 0, 2, 0 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        test_evalues = new double[][] {
            new double[] { 0, 0, BnUtils.ToRadians(-90) },
            new double[] { 0, BnUtils.ToRadians(90), 0 },
            new double[] { 0, 0, 0 }};
        bnaik.Compute(new double[] { 0, -2, 0 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        test_evalues = new double[][] {
            new double[] { 0, 0, BnUtils.ToRadians(180) },
            new double[] { 0, BnUtils.ToRadians(90), 0 },
            new double[] { 0, 0, 0 }};
        bnaik.Compute(new double[] { -2, 0, 0 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        test_evalues = new double[][] {
            new double[] { 0, 0, BnUtils.ToRadians(45) },
            new double[] { 0, BnUtils.ToRadians(45), 0 },
            new double[] { 0, BnUtils.ToRadians(45), 0 }};
        bnaik.Compute(new double[] { 1.210, 1.210, 0.70 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-2f, 1e-2f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-2f, 1e-2f));

        test_evalues = new double[][] {
            new double[] { 0, 0, BnUtils.ToRadians(45) },
            new double[] { 0, BnUtils.ToRadians(90), 0 },
            new double[] { 0, 0, 0 }};
        bnaik.Compute(new double[] { 1.416094183921814, 1.416094183921814, 0.0 }, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));
    }


    [Test]
    public void Test_BnRobotArm_MT()
    {

        // Arms along the Y axis
        BnMotionTracking_2Nodes bnmotiontrack = new BnMotionTracking_2Nodes(
            new double[] { 0, 0, 0 }, new double[] { 0, 1, 0 }, new double[] { 0, 1, 0 }, null, "cm");

        BnRobotIK_ArmZYY bnaik = new BnRobotIK_ArmZYY(
            0, 1, 1, null, "cm");

        // X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
        double[][] endpos = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 }};
        double[] node1_quat = new double[] { 0.92388, 0.382683, 0, 0 };
        double[] node2_quat = new double[] { 1, 0, 0, 0 };

        BnRobotArm_MT robotMT = new BnRobotArm_MT(bnmotiontrack, bnaik);

        // [90.         45.00005363 44.99993373]]
        double[][] test_evalues = new double[][] {
            new double[] { 0, 0, BnUtils.ToRadians(90) },
            new double[] { 0, BnUtils.ToRadians(45.00005363), 0 },
            new double[] { 0,  BnUtils.ToRadians(44.99993373), 0 }};
        double[][] test_ovalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 },
            new double[] { 0, 0, 0 }};

        robotMT.Compute(node1_quat, node2_quat, endpos, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        // ---------
        // Arms along the X axis
        bnmotiontrack = new BnMotionTracking_2Nodes(
            new double[] { 0, 0, 0 }, new double[] { 1, 0, 0 }, new double[] { 1, 0, 0 }, null, "cm");

        bnaik = new BnRobotIK_ArmZYY(
            0, 1, 1, null, "cm");

        // Y rotation 80, -20 -> [ 1.12019681930542, 0.0, 0.634331464767456 ]
        node1_quat = new double[] { 0.766044, 0, -0.642788, 0 };
        node2_quat = new double[] { 0.984808, 0, 0.173648, 0 };

        robotMT = new BnRobotArm_MT(bnmotiontrack, bnaik);

        // [0.         10. 100.]]
        test_evalues = new double[][] {
            new double[] { 0, 0, 0 },
            new double[] { 0, BnUtils.ToRadians(9.99994606), 0 },
            new double[] { 0,  BnUtils.ToRadians(100.00004607), 0 }};

        robotMT.Compute(node1_quat, node2_quat, endpos, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));
    }


    [Test]
    public void Test_BnRobotArm_MT_Constraints()
    {

        // Arms along the Y axis
        BnMotionTracking_2Nodes bnmotiontrack = new BnMotionTracking_2Nodes(
            new double[] { 0, 0, 0 }, new double[] { 0, 1, 0 }, new double[] { 0, 1, 0 }, null, "cm");

        BnRobotIK_ArmZYY bnaik = new BnRobotIK_ArmZYY(
            0, 1, 1,
            new double[][]{
                new double[]{ BnUtils.ToRadians(-45), BnUtils.ToRadians(45) },
                new double[]{ 0 , BnUtils.ToRadians(90)},
                new double[]{ 0 , BnUtils.ToRadians(90)}},
            "cm");

        double[][] endpos = new double[][] {
            new double[]{ 0, 0, 0 },
            new double[]{ 0, 0, 0 },
            new double[]{ 0, 0, 0 }};

        // X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
        double[] node1_quat = new double[] { 0.92388, 0.382683, 0, 0 };
        double[] node2_quat = new double[] { 1, 0, 0, 0 };

        BnRobotArm_MT robotMT = new BnRobotArm_MT(bnmotiontrack, bnaik);

        // [45.         45.00005363 44.99993373]]
        double[][] test_evalues = new double[][] {
            new double[]{ 0, 0, BnUtils.ToRadians(45) },
            new double[]{ 0, BnUtils.ToRadians(45.00005363), 0 },
            new double[]{ 0,  BnUtils.ToRadians(44.99993373), 0 }};
        double[][] test_ovalues = new double[][] {
            new double[]{ 0, 0, 0 },
            new double[]{ 0, 0, 0 },
            new double[]{ 0, 0, 0 }};

        robotMT.Compute(node1_quat, node2_quat, endpos, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));
        // ---------
        // Arms along the X axis
        bnmotiontrack = new BnMotionTracking_2Nodes(
            new double[] { 0, 0, 0 }, new double[] { 1, 0, 0 }, new double[] { 1, 0, 0 }, null, "cm");

        // The Robot IK will always assume as a starting position the arms to be pointing upwards
        bnaik = new BnRobotIK_ArmZYY(
            0, 1, 1,
            new double[][]{
                new double[]{ BnUtils.ToRadians(-90), BnUtils.ToRadians(90) },
                new double[]{ 0 , BnUtils.ToRadians(90)},
                new double[]{0 , BnUtils.ToRadians(90)}},
            "cm");

        // Z rotation 135
        // Y rotation 10, 20 -> [ -1.3624130487442017, 1.3624131679534912, -0.5175356268882751 ]
        node1_quat = new double[] { 0.381227, -0.080521, 0.033353, 0.920364 };
        node2_quat = new double[] { 0.37687, -0.16043, 0.066452, 0.909844 };

        robotMT = new BnRobotArm_MT(bnmotiontrack, bnaik);

        // [90.         90 30]]
        test_evalues = new double[][] {
            new double[]{ 0, 0, BnUtils.ToRadians(90) },
            new double[]{ 0, BnUtils.ToRadians(90), 0 },
            new double[]{ 0,  BnUtils.ToRadians(29.15184909), 0 }};

        robotMT.Compute(node1_quat, node2_quat, endpos, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        bnaik = new BnRobotIK_ArmZYY(
            0, 1, 1,
            new double[][]{
                new double[]{ BnUtils.ToRadians(-90), BnUtils.ToRadians(90) },
                new double[]{ 0 , BnUtils.ToRadians(100)},
                new double[]{0 , BnUtils.ToRadians(90)}},
            "cm");

        robotMT = new BnRobotArm_MT(bnmotiontrack, bnaik);

        // [90.         100 10]]
        test_evalues = new double[][] {
            new double[]{ 0, 0, BnUtils.ToRadians(90) },
            new double[]{ 0, BnUtils.ToRadians(100), 0 },
            new double[]{ 0,  BnUtils.ToRadians(9.9999254), 0 }};

        robotMT.Compute(node1_quat, node2_quat, endpos, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));

        bnaik = new BnRobotIK_ArmZYY(
            0, 1, 1,
            new double[][]{
                new double[]{ BnUtils.ToRadians(-90), BnUtils.ToRadians(90) },
                new double[]{ 0 , BnUtils.ToRadians(180)},
                new double[]{0 , BnUtils.ToRadians(90)}},
            "cm");

        robotMT = new BnRobotArm_MT(bnmotiontrack, bnaik);

        // [90.         100 10]]
        test_evalues = new double[][] {
            new double[]{ 0, 0, BnUtils.ToRadians(90) },
            new double[]{ 0, BnUtils.ToRadians(100.00035347), 0 },
            new double[]{ 0,  BnUtils.ToRadians(9.99922384), 0 }};

        robotMT.Compute(node1_quat, node2_quat, endpos, test_ovalues);
        Assert.That(ArraysAreClose(test_evalues[0], test_ovalues[0], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[1], test_ovalues[1], 1e-6f, 1e-6f));
        Assert.That(ArraysAreClose(test_evalues[2], test_ovalues[2], 1e-6f, 1e-6f));
    }

}

