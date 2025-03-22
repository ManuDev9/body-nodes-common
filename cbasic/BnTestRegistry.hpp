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

#ifndef BN_TEST_REGISTRY
#define BN_TEST_REGISTRY

#include <iostream>
#include <string>
#include <vector>

// Macro to define a test function
#define TEST_CASE(name) \
    void name(); \
    BnTestRegistry registry_##name(name, #name); \
    void name()

// Macro to assert a condition in the test
#define ASSERT_EQUAL(val1, val2) \
    do { \
        if (!(val1 == val2)) { \
            std::cout << __FILE__ << ":" << __LINE__ << ": Test failed: " << val1 << " != " << val2 << std::endl; \
            return; \
        } \
        else { \
          std::cout << __FILE__ << ":" << __LINE__ << ": Test Passed: " << val1 << " != " << val2 << std::endl; \
        } \
    } while (false)

// Macro to assert a condition in the test
#define ASSERT(val) \
    do { \
        if (!(val)) { \
            std::cout << __FILE__ << ":" << __LINE__ << ": Test failed." << std::endl; \
            return; \
        } \
        else { \
          std::cout << __FILE__ << ":" << __LINE__ << ": Test Passed. " << std::endl; \
        } \
    } while (false)

class BnTestRegistry {

private:
    BnTestRegistry() {};
  
public:
    BnTestRegistry(void (*testFunc)(), const std::string& testName) {
        getTestRegistry().add(testFunc, testName);
    }

    static BnTestRegistry& getTestRegistry() {
        static BnTestRegistry instance;
        return instance;
    }

    void add(void (*testFunc)(), const std::string& testName) {
        tests.push_back({testFunc, testName});
    }

    void runAllTests() {
        for (const auto& test : tests) {
            std::cout << "Running test: " << test.testName << std::endl;
            test.testFunc();
        }
    }

private:
    struct Test {
        void (*testFunc)();
        std::string testName;
    };

    std::vector<Test> tests;
};

#endif // BN_TEST_REGISTRY
