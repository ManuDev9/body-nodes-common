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


using System;

/*
 * dotnet new console
 * dotnet build
 * dotnet run
 */

public static class TestBnCommon
{
    // Define a test method
    public static void BnReorientAxisTest()
    {

        int[] test_io_axis = new int[]{ 3, 2, 1, 0 };
        int[] test_io_sign = new int[]{ -1, -1, -1, -1 };
        float[] test_ivalues = new float[]{ 3.0f, 8.0f, 2.0f, 10.2f };
        float[] test_ovalues = new float[]{ 3.0f, 8.0f, 2.0f, 10.2f };
        float[] test_evalues = new float[]{ -10.2f, -2.0f, -8.0f, -3.0f };

        BnReorientAxis test_obj = new BnReorientAxis();
        test_obj.config( test_io_axis, test_io_sign );
        test_obj.apply( test_ovalues );

        BnTestHelper.Assert(test_evalues[0] == test_ovalues[0]);
        BnTestHelper.Assert(test_evalues[1] == test_ovalues[1]);
        BnTestHelper.Assert(test_evalues[2] == test_ovalues[2]);
        BnTestHelper.Assert(test_evalues[3] == test_ovalues[3]);
    }

    public static void Main()
    {
        // Register the test method
        BnTestHelper.DefineTest(BnReorientAxisTest, nameof(BnReorientAxisTest));

        // Run all tests
        BnTestRegistry.Instance.RunAllTests();
    }
}

