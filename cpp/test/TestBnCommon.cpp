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

#include "BnAxisConfig.h"
#include "BnMotionTracking_2Nodes.h"
#include "BnQuaternion.h"
#include "BnRobotArm_MT.h"
#include "BnRobotIK_ArmZYY.h"
#include "BnUtils.h"

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

// C++ Standard Library headers
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// The nlohmann/json header
#include "json.hpp"

using json = nlohmann::json;

using namespace bodynodesdev::common;

// --- Helper Functions to Read Files (C++ style) ---
// Note: This replaces all the fopen/fseek/fread/malloc/free boilerplate.
std::string read_file_to_string(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        // Throw an exception that Catch2 can handle gracefully
        throw std::runtime_error("Failed to open file: " + path);
    }

    // Read the entire file content into a string stream
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// This helper function searches the header content for the macro and
// attempts to return its defined value as a string.
std::string extract_macro_value(const std::string &headerContent, const std::string &key) {

    // The macro definition structure we are looking for: #define BN_KEY VALUE
    std::string macroStart = "#define BN_" + key + " ";

    size_t startPos = headerContent.find(macroStart);

    if (startPos == std::string::npos) {
        // Macro not found (this shouldn't happen if the presence check passed,
        // but it's good defensive programming).
        return "";
    }

    // Start searching for the value just after the macro name and space
    size_t valueStart = startPos + macroStart.length();

    // Find the end of the definition line (look for newline or carriage return)
    size_t valueEnd = headerContent.find_first_of("\r\n", valueStart);

    // If no newline is found, assume the rest of the string is the value.
    if (valueEnd == std::string::npos) {
        valueEnd = headerContent.length();
    }

    // Return the substring containing the defined value, trimmed of leading/trailing spaces
    std::string value = headerContent.substr(valueStart, valueEnd - valueStart);

    // Simple trim (optional, but robust)
    size_t first = value.find_first_not_of(' ');
    size_t last = value.find_last_not_of(' ');

    if (std::string::npos == first) {
        return ""; // Value was only spaces
    }

    value = value.substr(first, (last - first + 1));
    if (value.length() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.length() - 2);
    }

    return value;
}

/**
 * @brief Asserts that two C-style arrays of doubles/floats are equal
 * within specified absolute and relative tolerances.
 * * @param ExpectedArray The array of expected values.
 * @param ActualArray The array of actual/observed values.
 * @param ArrayLength The number of elements to check.
 * @param AbsTolerance The absolute tolerance (Unity's delta, Catch2's margin).
 * @param RelTolerance The relative tolerance (Unity's error, Catch2's epsilon).
 */
#define TEST_ASSERT_DOUBLE_ARRAY(ExpectedArray, ActualArray, ArrayLength, AbsTolerance, RelTolerance)                  \
    do {                                                                                                               \
        using Catch::Matchers::WithinAbs;                                                                              \
        using Catch::Matchers::WithinRel;                                                                              \
                                                                                                                       \
        for (int i = 0; i < (ArrayLength); ++i) {                                                                      \
            const double actual_val = (ActualArray)[i];                                                                \
            const double expected_val = (ExpectedArray)[i];                                                            \
            INFO("Index [" << i << "]: Expected=" << expected_val << ", Actual=" << actual_val);                       \
            /* Check for the specific NaN case */                                                                      \
            if (std::isnan(expected_val) && std::isnan(actual_val)) {                                                  \
                /* Case 1: Both are NaN - Assertion passes */                                                          \
                SUCCEED("Index [" << i << "]: Expected NaN, Actual NaN (MATCH)");                                      \
            } else if (std::isnan(expected_val) != std::isnan(actual_val)) {                                           \
                /* Case 2: One is NaN, one is a number - Assertion Fails */                                            \
                FAIL("Index [" << i << "]: Mismatch. Expected " << (std::isnan(expected_val) ? "NaN" : "Number")       \
                               << ", Actual " << (std::isnan(actual_val) ? "NaN" : "Number"));                         \
            } else {                                                                                                   \
                /* Case 3: Both are numbers - Perform standard tolerance check */                                      \
                REQUIRE_THAT(actual_val, WithinAbs(expected_val, static_cast<double>(AbsTolerance)) ||                 \
                                             WithinRel(expected_val, static_cast<double>(RelTolerance)));              \
            }                                                                                                          \
        }                                                                                                              \
    } while (0)

#define TEST_ASSERT_FLOAT_ARRAY(ExpectedArray, ActualArray, ArrayLength, AbsTolerance, RelTolerance)                   \
    do {                                                                                                               \
        using Catch::Matchers::WithinAbs;                                                                              \
        using Catch::Matchers::WithinRel;                                                                              \
                                                                                                                       \
        for (int i = 0; i < (ArrayLength); ++i) {                                                                      \
            const float actual_val = (ActualArray)[i];                                                                 \
            const float expected_val = (ExpectedArray)[i];                                                             \
            INFO("Index [" << i << "]: Expected=" << expected_val << ", Actual=" << actual_val);                       \
            /* Check for the specific NaN case */                                                                      \
            if (std::isnan(expected_val) && std::isnan(actual_val)) {                                                  \
                /* Case 1: Both are NaN - Assertion passes */                                                          \
                SUCCEED("Index [" << i << "]: Expected NaN, Actual NaN (MATCH)");                                      \
            } else if (std::isnan(expected_val) != std::isnan(actual_val)) {                                           \
                /* Case 2: One is NaN, one is a number - Assertion Fails */                                            \
                FAIL("Index [" << i << "]: Mismatch. Expected " << (std::isnan(expected_val) ? "NaN" : "Number")       \
                               << ", Actual " << (std::isnan(actual_val) ? "NaN" : "Number"));                         \
            } else {                                                                                                   \
                REQUIRE_THAT((ActualArray)[i], WithinAbs((ExpectedArray)[i], static_cast<float>(AbsTolerance)) ||      \
                                                   WithinRel((ExpectedArray)[i], static_cast<float>(RelTolerance)));   \
            }                                                                                                          \
        }                                                                                                              \
    } while (0)

#define TEST_ASSERT_INT_ARRAY(ExpectedArray, ActualArray, ArrayLength)                                                 \
    for (int i = 0; i < (ArrayLength); ++i) {                                                                          \
        REQUIRE((ActualArray)[i] == (ExpectedArray)[i]);                                                               \
    }

////////////////////////////// TESTS

TEST_CASE("Test_BnConstants") {

    const std::string jsonPath = "../BNCONSTANTS.json";
    const std::string headerPath = "src/BnConstants.h";

    // Load JSON file
    std::string jsonContent;
    REQUIRE_NOTHROW(jsonContent = read_file_to_string(jsonPath));
    json allConstants;
    REQUIRE_NOTHROW(allConstants = json::parse(jsonContent));
    REQUIRE(allConstants.is_object() == true);

    // Load Header
    std::string bufferHeader;
    REQUIRE_NOTHROW(bufferHeader = read_file_to_string(headerPath));

    // Iterate JSON and Check Header Content
    std::vector<std::string> missingConstants;
    std::map<std::string, std::pair<std::string, std::string>> mismatchValues;

    const std::string prefix = "#define BN_";
    const std::string suffix = " ";
    for (json::iterator it = allConstants.begin(); it != allConstants.end(); ++it) {
        const std::string &key = it.key();

        // Filter: Skip keys starting with "__"
        if (key.rfind("__", 0) == 0) {
            continue;
        }

        // Construct the full string to search for
        std::string stringToSearch = prefix + key + suffix;

        // Check if the constructed string is NOT found in the header file
        if (bufferHeader.find(stringToSearch) == std::string::npos) {
            missingConstants.push_back(key);
        } else {
            std::string jsonValue = it.value().dump();

            // nlohmann::json::dump() will correctly quote strings and format numbers.
            // We need to strip the quotes if the JSON value is a string,
            // unless the C macro is also quoted (often it is not).

            // Simplification: Assume non-string values (numbers) are compared directly,
            // and string values in JSON are not quoted in the C macro.

            if (it.value().is_string()) {
                jsonValue = jsonValue.substr(1, jsonValue.length() - 2);
            }

            // Get the value from the Header
            std::string headerValue = extract_macro_value(bufferHeader, key);

            //  Compare the values
            if (jsonValue != headerValue) {
                mismatchValues[key] = {jsonValue, headerValue};
            }
        }
    }

    if (!missingConstants.empty()) {
        std::stringstream ss;
        ss << "Total " << missingConstants.size() << " constants found in JSON are missing in C header (" << headerPath
           << ").\n";
        ss << "Missing keys: ";
        for (const auto &key : missingConstants) {
            ss << key << ", ";
        }
        FAIL(ss.str());
    } else {
        // If the list is empty, assert that the size is 0 (optional, but clean)
        REQUIRE(missingConstants.empty());
    }

    if (!mismatchValues.empty()) {
        std::stringstream ss;
        ss << "Total " << mismatchValues.size() << " constant values found in JSON do not match values in C header.\n";
        ss << "Mismatches:\n";
        for (const auto &pair : mismatchValues) {
            ss << "  " << pair.first << ": JSON='" << pair.second.first << "', Header='" << pair.second.second << "'\n";
        }
        // Use FAIL to log all the specific errors
        FAIL(ss.str());
    } else {
        // The clean assertion
        REQUIRE(mismatchValues.empty());
    }
}

TEST_CASE("Test_BnQuaternion") {

    BnQuaternion q1(1, 2, 3, 4);
    BnQuaternion q2(0, 1, 0, 0);

    BnQuaternion q3 = q1.mul(q2);
    double test_ovalues[4];
    double const test_evalues1[4] = {-2, 1, 4, -3};
    q3.toList(test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1, test_ovalues, 4, 1e-9f, 1e-6f);

    // Conjugate
    double const test_evalues2[4] = {1, -2, -3, -4};
    q3 = q1.conjugate();
    q3.toList(test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2, test_ovalues, 4, 1e-9f, 1e-6f);

    // Inverse
    double const test_evalues3[4] = {0.03333333333333333, -0.06666666666666667, -0.1, -0.13333333333333333};
    q3 = q1.inverse();
    q3.toList(test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3, test_ovalues, 4, 1e-9f, 1e-6f);
}

TEST_CASE("Test_BnUtils") {

    double test_ovalues[3][3];
    BnUtils::blender_euler_to_rotation_matrix_rad(0.123, 2.12, -3.11, test_ovalues);

    double const test_evalues1[3][3] = {{0.52174769, -0.07324637, -0.8499496},
                                        {0.01648888, -0.99525533, 0.09589024},
                                        {-0.85294048, -0.06404523, -0.51806442}};
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[i], test_ovalues[i], 3, 0.001, 1e-2f);
    }

    BnUtils::blender_euler_to_rotation_matrix_degree(30, 40, 50, test_ovalues);
    double const test_evalues2[3][3] = {{0.49240388, -0.45682599, 0.74084306},
                                        {0.58682409, 0.80287234, 0.10504046},
                                        {-0.64278761, 0.38302222, 0.66341395}};
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[i], test_ovalues[i], 3, 0.001, 1e-2f);
    }

    double const test_ivalues3_1[3][3] = {{1, 2, 3}, {4, 5, 6}, {1, 2, 3}};
    double const test_ivalues3_2[3][3] = {{4, 5, 3}, {1, 5, 8}, {9, 1, 4}};
    double const test_evalues3[3][3] = {{33, 18, 31}, {75, 51, 76}, {33, 18, 31}};
    BnUtils::multiplyRotationMatrices(test_ivalues3_1, test_ivalues3_2, test_ovalues);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[i], test_ovalues[i], 3, 0.001, 1e-2f);
    }

    double test_ovalues_v4[4];
    BnAxisConfig axisConfig;
    axisConfig.config(1, -1, -1, 1, 0, 1, 3, 2);
    double const test_ivalues4[4] = {3, 4, 2, 1};
    double const test_evalues4[4] = {3.0, -4.0, -1.0, 2.0};
    BnQuaternion q4 = BnUtils::createQuanternion(axisConfig, test_ivalues4);
    q4.toList(test_ovalues_v4);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4, test_ovalues_v4, 4, 1e-9, 1e-6f);

    double firstQuatVals_1[4] = {0, 0, 0, 0};
    double valuesOut[4];
    BnUtils::transformSensorQuat((std::vector<double>{0.4, 0.3, 0.2, 0.3}).data(),                  // sensorQuatVals
                                 firstQuatVals_1, (std::vector<double>{0.4, 0.3, 0.2, 0.3}).data(), // startingQuatVals
                                 (std::vector<double>{0.4, 0.3, 0.2, 0.3}).data(),                  // envQuatVals
                                 (std::vector<int>{1, -1, -1, 1, 0, 1, 3, 2}).data(),               // pureAxisConfig
                                 valuesOut);
    double const test_evalues5_1[4] = {0.39999999999999997, 0.2999999999999999, 0.19999999999999998,
                                       0.29999999999999993};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_1, valuesOut, 4, 1e-9, 1e-6f);

    double const test_evalues5_2[4] = {1.0526315789473684, -0.7894736842105263, -0.5263157894736842,
                                       -0.7894736842105263};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_2, firstQuatVals_1, 4, 1e-9, 1e-6f);

    double firstQuatVals_2[4] = {0.4, 0.3, 0.2, 0.3};
    BnUtils::transformSensorQuat((std::vector<double>{0.4, 0.3, 0.2, 0.3}).data(),                  // sensorQuatVals
                                 firstQuatVals_2, (std::vector<double>{0.4, 0.3, 0.2, 0.3}).data(), // startingQuatVals
                                 (std::vector<double>{0.4, 0.3, 0.2, 0.3}).data(),                  // envQuatVals
                                 (std::vector<int>{1, -1, -1, 1, 0, 1, 3, 2}).data(),               // pureAxisConfig
                                 valuesOut);
    double const test_evalues5_3[4] = {0.048, -0.009999999999999974, -0.22799999999999995, 0.022000000000000006};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_3, valuesOut, 4, 1e-9, 1e-6f);
    double const test_evalues5_4[4] = {0.4, 0.3, 0.2, 0.3};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_4, firstQuatVals_2, 4, 1e-9, 1e-6f);
}

TEST_CASE("Test_BnAxisConfig") {

    int const test_io_axis[4] = {3, 2, 1, 0};
    int const test_io_sign[4] = {-1, -1, -1, -1};
    BnAxisConfig test_obj;
    test_obj.config(test_io_axis, test_io_sign);

    float test_ovaluesF[4] = {3.f, 4.5f, 2.f, 10.2f};
    float const test_evaluesF[4] = {-10.2f, -2.f, -4.5f, -3.f};
    test_obj.apply(test_ovaluesF);
    TEST_ASSERT_FLOAT_ARRAY(test_evaluesF, test_ovaluesF, 4, 1e-9f, 1e-6f);

    double test_ovaluesD[4] = {3, 4.5, 2, 10.2};
    double const test_evaluesD[4] = {-10.2, -2, -4.5, -3};
    test_obj.apply(test_ovaluesD);
    TEST_ASSERT_DOUBLE_ARRAY(test_evaluesD, test_ovaluesD, 4, 1e-9f, 1e-6f);

    //  ovalues are equal to ivalues for inplace operators
    int test_ovaluesI[4] = {3, 4, 2, 10};
    int const test_evaluesI[4] = {-10, -2, -4, -3};
    test_obj.apply(test_ovaluesI);
    TEST_ASSERT_INT_ARRAY(test_evaluesI, test_ovaluesI, 4);
}

TEST_CASE("Test_BnMotionTracking_2Nodes") {

    double const initialPosition[3] = {0, 0, 0};
    double const armVector1[3] = {10, 0, 0};
    double const armVector2[3] = {10, 0, 0};
    char const units[] = "cm";

    BnMotionTracking_2Nodes bnmotiontrack(initialPosition, armVector1, armVector2, NULL, units);

    double const test_node1_quat[4] = {0.9926, 0.0329, 0.0973, 0.0640};
    double const test_node2_quat[4] = {0.9583, -0.1367, -0.0595, -0.2439};
    double const test_evalues[3] = {18.468636171839087, -3.1761790635934757, -0.08354223767877755};

    double test_ovalues[3][3] = {
        {1, 3, 2},
        {1, 3, 2},
        {1, 2, 3},
    };
    bnmotiontrack.compute(test_node1_quat, test_node2_quat, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues, test_ovalues[2], 4, 1e-2f, 1e-2f);
}

TEST_CASE("Test_BnMotionTracking_2NodesConstraint") {

    double const locationConstraints[3][2] = {{10, 20}, {-5, 5}, {-5, 5}};
    char const units[] = "cm";

    BnMotionTracking_2Nodes bnmotiontrack((std::vector<double>{0, 0, 0}).data(),  // initialPosition
                                          (std::vector<double>{10, 0, 0}).data(), // armVector1
                                          (std::vector<double>{10, 0, 0}).data(), // armVector2
                                          locationConstraints, units);

    double const test_node1_quat[4] = {0.8504, 0.3678, -0.1840, 0.3281};
    double const test_node2_quat[4] = {0.9293, -0.0039, -0.2892, 0.2296};
    double const test_evalues[3] = {14.443218483410508, 5, 5};

    double test_ovalues[3][3] = {{1, 2, 3}, {1, 2, 3}, {1, 2, 3}};

    bnmotiontrack.compute(test_node1_quat, test_node2_quat, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues, test_ovalues[2], 4, 1e-2f, 1e-2f);
}

TEST_CASE("Test_BnRobotIK_ArmZYY") {

    double const test_endpoint[3] = {18.219124272891392, 3.8972461548699857, 1.6501078154541111};
    char const units[] = "cm";

    BnRobotIK_ArmZYY bnaik(0, 10, 10, NULL, units);

    double const test_evalues[3][3] = {
        {0, 0, 0.21073373345528476}, {0, 1.120530930230784, 0}, {0, 0.723883473845901, 0}};
    double test_ovalues[3][3] = {{1, 2, 3}, {1, 2, 3}, {1, 2, 3}};

    bnaik.compute(test_endpoint, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[2], test_ovalues[2], 3, 1e-2f, 1e-2f);
}

TEST_CASE("Test_BlenderSimpleLinksProj1") {

    double const initialPosition[3] = {0, 0, 2};
    double const armVector1[3] = {0, 1, 0};
    double const armVector2[3] = {0, 1, 0};
    char const units[] = "cm";

    BnMotionTracking_2Nodes bnmotiontrack(initialPosition, armVector1, armVector2, nullptr, units);

    // X rotation -90, -90
    double const node1_quat[4] = {0.707107, -0.707107, 0, 0};
    double const node2_quat[4] = {0.707107, -0.707107, 0, 0};

    double const test_evalues[3][3] = {
        {0, 0, 2},
        {0.0, -6.188980001819999e-07, 0.9999993811019998},
        {0.0, -1.2377960003639998e-06, -1.2377960003639998e-06},
    };

    double test_ovalues[3][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
    };

    bnmotiontrack.compute(node1_quat, node2_quat, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[0], test_ovalues[0], 3, 1e-3f, 1e-3f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[1], test_ovalues[1], 3, 1e-3f, 1e-3f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[2], test_ovalues[2], 3, 1e-3f, 1e-3f);
}

TEST_CASE("Test_BlenderSimpleLinksProj2") {

    // Testing how to setup the blender utility functions to correspond to what Blender is giving as output. We want
    // rotation XYZ
    double rot1[3][3];
    BnUtils::blender_euler_to_rotation_matrix_degree(45, 45, 45, rot1);
    double const vec1[3] = {0, 1, 0};
    double const test_evalues[3] = {-0.14644661, 0.85355339, 0.5};
    double vout[3];
    BnUtils::multiplyRotationMatrixWithVector(rot1, vec1, vout);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues, vout, 4, 1e-6f, 1e-5f);
}

TEST_CASE("Test_BlenderSimpleLinksProj3") {

    // Let's check the BnRobotIK_ArmZYY
    // The arms length are 0,1,1
    // For this type of test on Blender the Y is the python X axis
    // This is because Blender is forcing me to do this
    char const units[] = "cm";

    BnRobotIK_ArmZYY bnaik(0, 1, 1, NULL, units);

    double const test_evalues1[3][3] = {{0, 0, std::nan("")}, {0, 0, 0}, {0, 0, 0}};
    double test_ovalues[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double const test_endpoint1[3] = {0, 0, 2};
    bnaik.compute(test_endpoint1, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues2[3][3] = {{0, 0, 0}, {0, BnUtils::toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint2[3] = {2, 0, 0};
    bnaik.compute(test_endpoint2, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues3[3][3] = {{0, 0, 0}, {0, BnUtils::toRadians(45), 0}, {0, BnUtils::toRadians(45), 0}};
    double const test_endpoint3[3] = {1.711, 0.0, 0.703};
    bnaik.compute(test_endpoint3, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[2], test_ovalues[2], 3, 1e-2f, 1e-2f);

    double const test_evalues4[3][3] = {{0, 0, 0}, {0, BnUtils::toRadians(45), 0}, {0, 0, 0}};
    double const test_endpoint4[3] = {1.416, 0.0, 1.416};
    bnaik.compute(test_endpoint4, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues5[3][3] = {{0, 0, 0}, {0, BnUtils::toRadians(90), 0}, {0, BnUtils::toRadians(45), 0}};
    double const test_endpoint5[3] = {1.707, 0.0, -0.713};
    bnaik.compute(test_endpoint5, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5[2], test_ovalues[2], 3, 1e-2f, 1e-2f);

    double const test_evalues6[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, BnUtils::toRadians(135), 0}};
    double const test_endpoint6[3] = {0.713, 0.0, 0.281};
    bnaik.compute(test_endpoint6, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues6[0], test_ovalues[0], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues6[1], test_ovalues[1], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues6[2], test_ovalues[2], 3, 1e-1f, 1e-1f);

    double const test_evalues7[3][3] = {{0, 0, std::nan("")}, {0, 0, 0}, {0, BnUtils::toRadians(180), 0}};
    double const test_endpoint7[3] = {0.0, 0.0, 0.0};
    bnaik.compute(test_endpoint7, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues7[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues7[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues7[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues8[3][3] = {{0, 0, 0}, {0, BnUtils::toRadians(-60), 0}, {0, BnUtils::toRadians(150), 0}};
    double const test_endpoint8[3] = {0.1472482681274414, 0.0, 0.497156023979187};
    bnaik.compute(test_endpoint8, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues8[0], test_ovalues[0], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues8[1], test_ovalues[1], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues8[2], test_ovalues[2], 3, 1e-1f, 1e-1f);

    double const test_evalues9[3][3] = {{0, 0, BnUtils::toRadians(90)}, {0, BnUtils::toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint9[3] = {0, 2, 0};
    bnaik.compute(test_endpoint9, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues9[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues9[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues9[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues10[3][3] = {{0, 0, BnUtils::toRadians(-90)}, {0, BnUtils::toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint10[3] = {0, -2, 0};
    bnaik.compute(test_endpoint10, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues10[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues10[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues10[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues11[3][3] = {{0, 0, BnUtils::toRadians(180)}, {0, BnUtils::toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint11[3] = {-2, 0, 0};
    bnaik.compute(test_endpoint11, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues11[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues11[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues11[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues12[3][3] = {
        {0, 0, BnUtils::toRadians(45)}, {0, BnUtils::toRadians(45), 0}, {0, BnUtils::toRadians(45), 0}};
    double const test_endpoint12[3] = {1.210, 1.210, 0.70};
    bnaik.compute(test_endpoint12, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues12[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues12[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues12[2], test_ovalues[2], 3, 1e-2f, 1e-2f);

    double const test_evalues13[3][3] = {{0, 0, BnUtils::toRadians(45)}, {0, BnUtils::toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint13[3] = {1.416094183921814, 1.416094183921814, 0.0};
    bnaik.compute(test_endpoint13, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues13[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues13[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues13[2], test_ovalues[2], 3, 1e-6f, 1e-6f);
}

TEST_CASE("Test_BnRobotArm_MT") {

    // Arms along the Y axis
    double const initialPosition_1[3] = {0, 0, 0};
    double const armVector1_1[3] = {0, 1, 0};
    double const armVector2_1[3] = {0, 1, 0};
    char const units[] = "cm";

    std::unique_ptr<BnMotionTracking_2Nodes> bnmotiontrack_ptr(
        new BnMotionTracking_2Nodes(initialPosition_1, armVector1_1, armVector2_1, nullptr, units));

    std::unique_ptr<BnRobotIK_ArmZYY> bnaik_ptr(new BnRobotIK_ArmZYY(0, 1, 1, nullptr, units));

    BnRobotArm_MT robotMT(std::move(bnmotiontrack_ptr), std::move(bnaik_ptr));

    // X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    double endpos[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double const node1_quat1[4] = {0.92388, 0.382683, 0, 0};
    double const node2_quat1[4] = {1, 0, 0, 0};

    // [90.         45.00005363 44.99993373]]
    double const test_evalues1[3][3] = {{0, 0, BnUtils::toRadians(90)},
                                        {0, BnUtils::toRadians(45.00005363), 0},
                                        {0, BnUtils::toRadians(44.99993373), 0}};
    double test_ovalues[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    robotMT.compute(node1_quat1, node2_quat1, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    // ---------
    // Arms along the X axis
    double const initialPosition_2[3] = {0, 0, 0};
    double const armVector1_2[3] = {1, 0, 0};
    double const armVector2_2[3] = {1, 0, 0};

    bnmotiontrack_ptr = std::unique_ptr<BnMotionTracking_2Nodes>(
        new BnMotionTracking_2Nodes(initialPosition_2, armVector1_2, armVector2_2, nullptr, units));

    bnaik_ptr = std::unique_ptr<BnRobotIK_ArmZYY>(new BnRobotIK_ArmZYY(0, 1, 1, nullptr, units));

    // Y rotation 80, -20 -> [ 1.12019681930542, 0.0, 0.634331464767456 ]
    double const node1_quat2[4] = {0.766044, 0, -0.642788, 0};
    double const node2_quat2[4] = {0.984808, 0, 0.173648, 0};

    robotMT = BnRobotArm_MT(std::move(bnmotiontrack_ptr), std::move(bnaik_ptr));

    // [0.         10. 100.]]
    double const test_evalues2[3][3] = {
        {0, 0, 0}, {0, BnUtils::toRadians(9.99994606), 0}, {0, BnUtils::toRadians(100.00004607), 0}};

    robotMT.compute(node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[2], test_ovalues[2], 3, 1e-6f, 1e-6f);
}

TEST_CASE("Test_BnRobotArm_MT_Constraints") {

    // Arms along the Y axis
    double const initialPosition_1[3] = {0, 0, 0};
    double const armVector1_1[3] = {0, 1, 0};
    double const armVector2_1[3] = {0, 1, 0};
    char const units[] = "cm";

    std::unique_ptr<BnMotionTracking_2Nodes> bnmotiontrack_ptr(
        new BnMotionTracking_2Nodes(initialPosition_1, armVector1_1, armVector2_1, nullptr, units));

    double const anglesConstraints1[3][2] = {
        {BnUtils::toRadians(-45), BnUtils::toRadians(45)}, {0, BnUtils::toRadians(90)}, {0, BnUtils::toRadians(90)}};
    std::unique_ptr<BnRobotIK_ArmZYY> bnaik_ptr(new BnRobotIK_ArmZYY(0, 1, 1, anglesConstraints1, units));

    BnRobotArm_MT robotMT(std::move(bnmotiontrack_ptr), std::move(bnaik_ptr));

    // X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    double endpos[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double const node1_quat1[4] = {0.92388, 0.382683, 0, 0};
    double const node2_quat1[4] = {1, 0, 0, 0};

    // [45.         45.00005363 44.99993373]]
    double const test_evalues1[3][3] = {{0, 0, BnUtils::toRadians(45)},
                                        {0, BnUtils::toRadians(45.00005363), 0},
                                        {0, BnUtils::toRadians(44.99993373), 0}};
    double test_ovalues[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    robotMT.compute(node1_quat1, node2_quat1, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    // ---------
    // Arms along the X axis
    double const initialPosition_2[3] = {0, 0, 0};
    double const armVector1_2[3] = {1, 0, 0};
    double const armVector2_2[3] = {1, 0, 0};

    bnmotiontrack_ptr = std::unique_ptr<BnMotionTracking_2Nodes>(
        new BnMotionTracking_2Nodes(initialPosition_2, armVector1_2, armVector2_2, NULL, units));

    double const anglesConstraints2[3][2] = {
        {BnUtils::toRadians(-90), BnUtils::toRadians(90)}, {0, BnUtils::toRadians(90)}, {0, BnUtils::toRadians(90)}};
    bnaik_ptr = std::unique_ptr<BnRobotIK_ArmZYY>(new BnRobotIK_ArmZYY(0, 1, 1, anglesConstraints2, units));

    robotMT = BnRobotArm_MT(std::move(bnmotiontrack_ptr), std::move(bnaik_ptr));

    // Z rotation 135
    // Y rotation 10, 20 -> [ -1.3624130487442017, 1.3624131679534912, -0.5175356268882751 ]
    double const node1_quat2[4] = {0.381227, -0.080521, 0.033353, 0.920364};
    double const node2_quat2[4] = {0.37687, -0.16043, 0.066452, 0.909844};

    // [90.         90 30]]
    double const test_evalues2[3][3] = {
        {0, 0, BnUtils::toRadians(90)}, {0, BnUtils::toRadians(90), 0}, {0, BnUtils::toRadians(29.15184909), 0}};

    robotMT.compute(node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const anglesConstraints3[3][2] = {
        {BnUtils::toRadians(-90), BnUtils::toRadians(90)}, {0, BnUtils::toRadians(100)}, {0, BnUtils::toRadians(90)}};
    bnaik_ptr = std::unique_ptr<BnRobotIK_ArmZYY>(new BnRobotIK_ArmZYY(0, 1, 1, anglesConstraints3, units));

    robotMT.setRobotIK(std::move(bnaik_ptr));

    // [90.         100 10]]
    double const test_evalues3[3][3] = {
        {0, 0, BnUtils::toRadians(90)}, {0, BnUtils::toRadians(100), 0}, {0, BnUtils::toRadians(9.9999254), 0}};

    robotMT.compute(node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const anglesConstraints4[3][2] = {
        {BnUtils::toRadians(-90), BnUtils::toRadians(90)}, {0, BnUtils::toRadians(180)}, {0, BnUtils::toRadians(90)}};
    bnaik_ptr = std::unique_ptr<BnRobotIK_ArmZYY>(new BnRobotIK_ArmZYY(0, 1, 1, anglesConstraints4, units));

    robotMT.setRobotIK(std::move(bnaik_ptr));

    // [90.         100 10]]
    double const test_evalues4[3][3] = {{0, 0, BnUtils::toRadians(90)},
                                        {0, BnUtils::toRadians(100.00035347), 0},
                                        {0, BnUtils::toRadians(9.99922384), 0}};

    robotMT.compute(node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[2], test_ovalues[2], 3, 1e-6f, 1e-6f);
}