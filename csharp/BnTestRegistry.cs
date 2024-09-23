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
using System.Collections.Generic;

// Define a delegate for test methods
public delegate void BnTestDelegate();

public class BnTestRegistry
{
    // Singleton instance of TestRegistry
    private static BnTestRegistry _instance = new BnTestRegistry();
    public static BnTestRegistry Instance => _instance;

    private List<Test> _tests = new List<Test>();

    // Private constructor for singleton pattern
    private BnTestRegistry() { }

    // Add a test method to the registry
    public void Add(BnTestDelegate testMethod, string testName)
    {
        _tests.Add(new Test(testMethod, testName));
    }

    // Run all registered tests
    public void RunAllTests()
    {
        foreach (var test in _tests)
        {
            Console.WriteLine($"Running test: {test.TestName}");
            test.TestMethod();
        }
    }

    // Internal class to hold test methods and names
    private class Test
    {
        public BnTestDelegate TestMethod { get; }
        public string TestName { get; }

        public Test(BnTestDelegate testMethod, string testName)
        {
            TestMethod = testMethod;
            TestName = testName;
        }
    }
}

// Helper class for defining tests and assertions
public static class BnTestHelper
{
    // Macro equivalent for defining a test method
    public static void DefineTest(BnTestDelegate testMethod, string testName)
    {
        BnTestRegistry.Instance.Add(testMethod, testName);
    }

    // Macro equivalent for assertions
    public static void Assert(bool condition)
    {
        if (!condition)
        {
            throw new Exception("\tFailed: " + new System.Diagnostics.StackTrace(true).ToString());
        }
        else
        {
            Console.WriteLine("\tPassed");
        }
    }

    public static bool areArraysClose(float[] actual, float[] expected, float absError, float relError) {
        if (actual.Length != expected.Length) {
            throw new Exception("Arrays must be of the same length.");
        }

        for (int i = 0; i < actual.Length; i++) {
            float diff = Math.Abs(actual[i] - expected[i]);
            float expectedValue = Math.Abs(expected[i]);

            // Check if the difference is within absolute or relative error bounds
            if (diff > absError && (expectedValue == 0 || diff / expectedValue > relError)) {
                return false;
            }
        }

        return true;
    }
}
