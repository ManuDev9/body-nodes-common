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

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define UNITY_INCLUDE_DOUBLE
#include "cJSON.h"
#include "unity.h"

// Finds the value defined after a macro key in the header content.
// Returns a dynamically allocated string containing the value, or NULL on
// failure.
char *extract_macro_value_c(const char *headerContent, const char *key) {
    const char *prefix = "#define BN_";
    const char *suffix = " ";

    // Construct the search string: #define BN_KEY
    size_t search_len = strlen(prefix) + strlen(key) + strlen(suffix) + 1;
    char *search_string = (char *)malloc(search_len);
    if (search_string == NULL)
        return NULL;

    snprintf(search_string, search_len, "%s%s%s", prefix, key, suffix);

    // 1. Find the start of the macro definition
    char *start_ptr = strstr(headerContent, search_string);
    free(search_string); // Done with the search string

    if (start_ptr == NULL)
        return NULL; // Macro not found

    // 2. Locate the start of the VALUE
    char *value_start = start_ptr + strlen(prefix) + strlen(key) + strlen(suffix);

    // 3. Find the end of the line (where the value ends)
    char *value_end = value_start;
    while (*value_end != '\n' && *value_end != '\r' && *value_end != '\0') {
        value_end++;
    }

    // 4. Extract the raw value string
    size_t raw_len = value_end - value_start;
    char *raw_value = (char *)malloc(raw_len + 1);
    if (raw_value == NULL)
        return NULL;
    strncpy(raw_value, value_start, raw_len);
    raw_value[raw_len] = '\0';

    // 5. Trim leading and trailing spaces
    char *trimmed_start = raw_value;
    while (isspace((unsigned char)*trimmed_start))
        trimmed_start++;

    char *trimmed_end = raw_value + strlen(raw_value) - 1;
    while (trimmed_end > trimmed_start && isspace((unsigned char)*trimmed_end))
        trimmed_end--;
    *(trimmed_end + 1) = '\0';

    // 6. Check and strip outer quotes (for string constants)
    char *final_value = trimmed_start;
    size_t final_len = strlen(final_value);

    if (final_len >= 2 && final_value[0] == '"' && final_value[final_len - 1] == '"') {
        // Create new string without quotes (need new allocation for safety)
        char *unquoted_value = (char *)malloc(final_len - 1);
        if (unquoted_value == NULL) {
            free(raw_value);
            return NULL;
        }

        strncpy(unquoted_value, final_value + 1, final_len - 2);
        unquoted_value[final_len - 2] = '\0';

        // Use the unquoted string as the final result
        free(raw_value);
        return unquoted_value;
    }

    // Otherwise, return the original trimmed string
    char *result = strdup(final_value);
    free(raw_value);
    return result;
}

/**
 * Checks if two arrays are close to each other within given absolute and
 * relative tolerances.
 *
 * @param actual Array of actual values.
 * @param expected Array of expected values.
 * @param length Lenght of both arrays
 * @param absError Absolute error tolerance.
 * @param relError Relative error tolerance.
 * @return True if arrays are close within tolerances, false otherwise.
 */
#define TEST_ASSERT_FLOAT_ARRAY(expected, actual, length, absErr, relErr)                                              \
    do {                                                                                                               \
        for (uint64_t _i = 0; _i < (length); _i++) {                                                                   \
            float _e = (expected)[_i];                                                                                 \
            float _a = (actual)[_i];                                                                                   \
            float _diff = fabsf(_a - _e);                                                                              \
            float _absE = fabsf(_e);                                                                                   \
                                                                                                                       \
            float _tol = (absErr);                                                                                     \
            if (_absE > 0.0f) {                                                                                        \
                float _relTol = _absE * (relErr);                                                                      \
                if (_relTol > _tol)                                                                                    \
                    _tol = _relTol;                                                                                    \
            }                                                                                                          \
                                                                                                                       \
            TEST_ASSERT_FLOAT_WITHIN(_tol, _e, _a);                                                                    \
        }                                                                                                              \
    } while (0)

#define TEST_ASSERT_DOUBLE_ARRAY(expected, actual, length, absErr, relErr)                                             \
    do {                                                                                                               \
        for (uint64_t _i = 0; _i < (length); _i++) {                                                                   \
            double _e = (expected)[_i];                                                                                \
            double _a = (actual)[_i];                                                                                  \
            double _absE = fabs(_e);                                                                                   \
                                                                                                                       \
            double _tol = (absErr);                                                                                    \
            double _relTol = _absE * (relErr);                                                                         \
            if (_relTol > _tol)                                                                                        \
                _tol = _relTol;                                                                                        \
                                                                                                                       \
            TEST_ASSERT_DOUBLE_WITHIN(_tol, _e, _a);                                                                   \
        }                                                                                                              \
    } while (0)

//////////// TESTS /////////////////

void Test_BnConstants(void) {

    const char *jsonPath = "../BNCONSTANTS.json";

    FILE *fp_json = fopen(jsonPath, "rb");
    if (fp_json == NULL) {
        TEST_FAIL_MESSAGE("Failed to open BNCONSTANTS.json. Check path/permissions.");
    }

    fseek(fp_json, 0, SEEK_END);
    long fsize = ftell(fp_json);
    fseek(fp_json, 0, SEEK_SET);

    char *jsonContent = (char *)malloc(fsize + 1);
    if (jsonContent == NULL) {
        fclose(fp_json);
        TEST_FAIL_MESSAGE("Memory allocation failed for JSON content.");
    }

    fread(jsonContent, 1, fsize, fp_json);
    fclose(fp_json);
    jsonContent[fsize] = '\0';

    cJSON *allConstants = cJSON_Parse(jsonContent);
    free(jsonContent);

    if (allConstants == NULL) {
        char msg[256];
        snprintf(msg, sizeof(msg), "Failed to parse JSON. Error at: %s", cJSON_GetErrorPtr());
        TEST_FAIL_MESSAGE(msg);
    }
    TEST_ASSERT_TRUE_MESSAGE(cJSON_IsObject(allConstants), "JSON root is not an object (expected dictionary).");

    const char *headerPath = "src/BnConstants.h";
    FILE *fp_header = fopen(headerPath, "r");

    if (fp_header == NULL) {
        TEST_FAIL_MESSAGE("FATAL: Failed to open BnConstants.h. Check file path "
                          "and permissions.");
    }

    fseek(fp_header, 0, SEEK_END);
    long file_size = ftell(fp_header);
    fseek(fp_header, 0, SEEK_SET);

    char *buffer_header = (char *)malloc(file_size + 1);
    if (buffer_header == NULL) {
        perror("Error allocating memory");
        fclose(fp_header);
        TEST_FAIL_MESSAGE("Memory allocation failed for buffer BnConstants.h");
    }

    size_t read_count = fread(buffer_header, 1, file_size, fp_header);
    buffer_header[file_size] = '\0';
    fclose(fp_header);

    int missing_count = 0;
    int mismatch_count = 0;
    cJSON *json_item = allConstants->child;

    const char *prefix = "#define BN_";
    const char *suffix = " ";
    size_t prefix_len = strlen(prefix);
    size_t suffix_len = strlen(suffix);

    while (json_item != NULL) {
        const char *key = json_item->string;

        // Filter: Skip keys starting with "__"
        if (strncmp(key, "__", 2) == 0) {
            json_item = json_item->next;
            continue;
        }

        size_t total_len = prefix_len + strlen(key) + suffix_len + 1; // +1 for '\0'

        char *string_to_seach = (char *)malloc(total_len);

        // Check if memory allocation failed
        if (string_to_seach == NULL) {
            perror("Failed to allocate memory for string_to_seach");
            free(buffer_header);
            cJSON_Delete(allConstants);
            return;
        }

        int chars_written = snprintf(string_to_seach, total_len, "%s%s%s", prefix, key, suffix);

        if (chars_written < 0 || (size_t)chars_written >= total_len) {
            fprintf(stderr, "Error or truncation during string construction.\n");
            free(string_to_seach);
            free(buffer_header);
            cJSON_Delete(allConstants);
            return;
        }
        if (strstr(buffer_header, string_to_seach) == NULL) {
            printf("    [MISSING] Constant: %s\n", key);
            missing_count++;
        } else {

            // Get the value from the Header file
            char *header_value = extract_macro_value_c(buffer_header, key);
            if (header_value == NULL) {
                // If the helper failed to allocate memory or find the value
                // (shouldn't happen if strstr passed), skip/fail.
                mismatch_count++;
                free(string_to_seach);
                json_item = json_item->next;
                continue;
            }

            // Get the value from the JSON item
            char json_value_buffer[256] = {0};

            if (cJSON_IsString(json_item)) {
                // For strings, use the stored valuestring directly
                strncpy(json_value_buffer, json_item->valuestring, sizeof(json_value_buffer) - 1);

            } else if (cJSON_IsNumber(json_item)) {
                // For numbers, convert to a string for comparison
                snprintf(json_value_buffer, sizeof(json_value_buffer), "%.f", json_item->valuedouble);

            } else {
                // Handle other types (e.g., bool/null/array) if necessary
                strncpy(json_value_buffer, "UNSUPPORTED_TYPE", sizeof(json_value_buffer) - 1);
            }

            // Compare the strings
            if (strcmp(header_value, json_value_buffer) != 0) {
                printf("    [MISMATCH] %s: JSON='%s', Header='%s'\n", key, json_value_buffer, header_value);
                mismatch_count++;
            }

            free(header_value); // Clean up helper function allocation
        }

        free(string_to_seach); // Clean up allocation in the loop
        json_item = json_item->next;
    }

    free(buffer_header);
    cJSON_Delete(allConstants);

    // Assert that no constants were missing
    char msg[128];
    snprintf(msg, sizeof(msg), "Total %d constants found in JSON are missing in C code.", missing_count);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, missing_count, msg);

    // Asser the values are all identical
    char msg_mismatch[128];
    snprintf(msg_mismatch, sizeof(msg_mismatch), "Total %d constants found in JSON have mismatching values in C code.",
             mismatch_count);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, mismatch_count, msg_mismatch);
}

void Test_BnQuaternion(void) {

    BnQuaternion_t q1 = BnQuaternion_create_wxyz(1, 2, 3, 4);
    BnQuaternion_t q2 = BnQuaternion_create_wxyz(0, 1, 0, 0);

    BnQuaternion_t q3 = BnQuaternion_mul(&q1, &q2);
    double test_ovalues[4];
    double const test_evalues1[4] = {-2, 1, 4, -3};
    BnQuaternion_to_list(&q3, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1, test_ovalues, 4, 1e-9f, 1e-6f);

    // Conjugate
    double const test_evalues2[4] = {1, -2, -3, -4};
    q3 = BnQuaternion_conjugate(&q1);
    BnQuaternion_to_list(&q3, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2, test_ovalues, 4, 1e-9f, 1e-6f);

    // Inverse
    double const test_evalues3[4] = {0.03333333333333333, -0.06666666666666667, -0.1, -0.13333333333333333};
    q3 = BnQuaternion_inverse(&q1);
    BnQuaternion_to_list(&q3, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3, test_ovalues, 4, 1e-9f, 1e-6f);
}

void Test_BnUtils(void) {

    double test_ovalues[3][3];
    BnUtils_blender_euler_to_rotation_matrix_rad(0.123, 2.12, -3.11, test_ovalues);

    double const test_evalues1[3][3] = {{0.52174769, -0.07324637, -0.8499496},
                                        {0.01648888, -0.99525533, 0.09589024},
                                        {-0.85294048, -0.06404523, -0.51806442}};
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[i], test_ovalues[i], 3, 0.001, 1e-2f);
    }

    BnUtils_blender_euler_to_rotation_matrix_degree(30, 40, 50, test_ovalues);
    double const test_evalues2[3][3] = {{0.49240388, -0.45682599, 0.74084306},
                                        {0.58682409, 0.80287234, 0.10504046},
                                        {-0.64278761, 0.38302222, 0.66341395}};
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[i], test_ovalues[i], 3, 0.001, 1e-2f);
    }

    double const test_ivalues3_1[3][3] = {{1, 2, 3}, {4, 5, 6}, {1, 2, 3}};
    double const test_ivalues3_2[3][3] = {{4, 5, 3}, {1, 5, 8}, {9, 1, 4}};
    double const test_evalues3[3][3] = {{33, 18, 31}, {75, 51, 76}, {33, 18, 31}};
    BnUtils_multiplyRotationMatrices(test_ivalues3_1, test_ivalues3_2, test_ovalues);
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[i], test_ovalues[i], 3, 0.001, 1e-2f);
    }

    double test_ovalues_v4[4];
    BnAxisConfig_t axisConfig;
    BnAxisConfig_config_vals(&axisConfig, 1, -1, -1, 1, 0, 1, 3, 2);
    double const test_ivalues4[4] = {3, 4, 2, 1};
    double const test_evalues4[4] = {3.0, -4.0, -1.0, 2.0};
    BnQuaternion_t q4 = BnUtils_createQuanternion(&axisConfig, test_ivalues4);
    BnQuaternion_to_list(&q4, test_ovalues_v4);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4, test_ovalues_v4, 4, 1e-9, 1e-6f);

    double const sensorQuatVals[4] = {0.4, 0.3, 0.2, 0.3};
    double firstQuatVals_1[4] = {0, 0, 0, 0};
    double const startingQuatVals[4] = {0.4, 0.3, 0.2, 0.3};
    double const envQuatVals[4] = {0.4, 0.3, 0.2, 0.3};
    int const pureAxisConfig[8] = {1, -1, -1, 1, 0, 1, 3, 2};
    double valuesOut[4];
    BnUtils_transformSensorQuat(sensorQuatVals, firstQuatVals_1, startingQuatVals, envQuatVals, pureAxisConfig,
                                valuesOut);
    double const test_evalues5_1[4] = {0.39999999999999997, 0.2999999999999999, 0.19999999999999998,
                                       0.29999999999999993};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_1, valuesOut, 4, 1e-9, 1e-6f);

    double const test_evalues5_2[4] = {1.0526315789473684, -0.7894736842105263, -0.5263157894736842,
                                       -0.7894736842105263};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_2, firstQuatVals_1, 4, 1e-9, 1e-6f);

    double firstQuatVals_2[4] = {0.4, 0.3, 0.2, 0.3};
    BnUtils_transformSensorQuat(sensorQuatVals, firstQuatVals_2, startingQuatVals, envQuatVals, pureAxisConfig,
                                valuesOut);
    double const test_evalues5_3[4] = {0.048, -0.009999999999999974, -0.22799999999999995, 0.022000000000000006};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_3, valuesOut, 4, 1e-9, 1e-6f);
    double const test_evalues5_4[4] = {0.4, 0.3, 0.2, 0.3};
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5_4, firstQuatVals_2, 4, 1e-9, 1e-6f);
}

void Test_BnAxisConfig(void) {

    int const test_io_axis[4] = {3, 2, 1, 0};
    int const test_io_sign[4] = {-1, -1, -1, -1};
    BnAxisConfig_t test_obj;
    BnAxisConfig_config(&test_obj, test_io_axis, test_io_sign);

    float test_ovaluesF[4] = {3.f, 4.5f, 2.f, 10.2f};
    float const test_evaluesF[4] = {-10.2f, -2.f, -4.5f, -3.f};
    BnAxisConfig_apply_float(&test_obj, test_ovaluesF);
    TEST_ASSERT_FLOAT_ARRAY(test_evaluesF, test_ovaluesF, 4, 1e-9f, 1e-6f);

    double test_ovaluesD[4] = {3, 4.5, 2, 10.2};
    double const test_evaluesD[4] = {-10.2, -2, -4.5, -3};
    BnAxisConfig_apply_double(&test_obj, test_ovaluesD);
    TEST_ASSERT_DOUBLE_ARRAY(test_evaluesD, test_ovaluesD, 4, 1e-9f, 1e-6f);

    //  ovalues are equal to ivalues for inplace operators
    int test_ovaluesI[4] = {3, 4, 2, 10};
    int const test_evaluesI[4] = {-10, -2, -4, -3};
    BnAxisConfig_apply_int(&test_obj, test_ovaluesI);
    TEST_ASSERT_EQUAL_INT_ARRAY(test_evaluesI, test_ovaluesI, 4);
}

void Test_BnMotionTracking_2Nodes(void) {

    double const initialPosition[3] = {0, 0, 0};
    double const armVector1[3] = {10, 0, 0};
    double const armVector2[3] = {10, 0, 0};
    char const units[] = "cm";

    BnMotionTracking_2Nodes_t bnmotiontrack =
        BnMotionTracking_2Nodes_create(initialPosition, armVector1, armVector2, NULL, units);

    double const test_node1_quat[4] = {0.9926, 0.0329, 0.0973, 0.0640};
    double const test_node2_quat[4] = {0.9583, -0.1367, -0.0595, -0.2439};
    double const test_evalues[3] = {18.468636171839087, -3.1761790635934757, -0.08354223767877755};

    double test_ovalues[3][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
    };
    BnMotionTracking_2Nodes_compute(&bnmotiontrack, test_node1_quat, test_node2_quat, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues, test_ovalues[2], 4, 1e-2f, 1e-2f);
}

void Test_BnMotionTracking_2NodesConstraint(void) {

    double const initialPosition[3] = {0, 0, 0};
    double const armVector1[3] = {10, 0, 0};
    double const armVector2[3] = {10, 0, 0};
    double const locationConstraints[3][2] = {{10, 20}, {-5, 5}, {-5, 5}};
    char const units[] = "cm";

    BnMotionTracking_2Nodes_t bnmotiontrack =
        BnMotionTracking_2Nodes_create(initialPosition, armVector1, armVector2, locationConstraints, units);

    double const test_node1_quat[4] = {0.8504, 0.3678, -0.1840, 0.3281};
    double const test_node2_quat[4] = {0.9293, -0.0039, -0.2892, 0.2296};
    double const test_evalues[3] = {14.443218483410508, 5, 5};

    double test_ovalues[3][3] = {{1, 2, 3}, {1, 2, 3}, {1, 2, 3}};

    BnMotionTracking_2Nodes_compute(&bnmotiontrack, test_node1_quat, test_node2_quat, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues, test_ovalues[2], 4, 1e-2f, 1e-2f);
}

void Test_BnRobotIK_ArmZYY(void) {

    double const test_endpoint[3] = {18.219124272891392, 3.8972461548699857, 1.6501078154541111};
    char const units[] = "cm";

    BnRobotIK_ArmZYY_t bnaik = BnRobotIK_ArmZYY_create(0, 10, 10, NULL, units);

    double const test_evalues[3][3] = {
        {0, 0, 0.21073373345528476}, {0, 1.120530930230784, 0}, {0, 0.723883473845901, 0}};
    double test_ovalues[3][3] = {{1, 2, 3}, {1, 2, 3}, {1, 2, 3}};

    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[2], test_ovalues[2], 3, 1e-2f, 1e-2f);
}

void Test_BlenderSimpleLinksProj1(void) {

    double const initialPosition[3] = {0, 0, 2};
    double const armVector1[3] = {0, 1, 0};
    double const armVector2[3] = {0, 1, 0};
    char const units[] = "cm";

    BnMotionTracking_2Nodes_t bnmotiontrack =
        BnMotionTracking_2Nodes_create(initialPosition, armVector1, armVector2, NULL, units);

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

    BnMotionTracking_2Nodes_compute(&bnmotiontrack, node1_quat, node2_quat, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[0], test_ovalues[0], 3, 1e-3f, 1e-3f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[1], test_ovalues[1], 3, 1e-3f, 1e-3f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues[2], test_ovalues[2], 3, 1e-3f, 1e-3f);
}

void Test_BlenderSimpleLinksProj2(void) {
    // Testing how to setup the blender utility functions to correspond to what
    // Blender is giving as output. We want rotation XYZ
    double rot1[3][3];
    BnUtils_blender_euler_to_rotation_matrix_degree(45, 45, 45, rot1);
    double const vec1[3] = {0, 1, 0};
    double const test_evalues[3] = {-0.14644661, 0.85355339, 0.5};
    double vout[3];
    BnUtils_multiplyRotationMatrixWithVector(rot1, vec1, vout);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues, vout, 4, 1e-6f, 1e-5f);
}

void Test_BlenderSimpleLinksProj3(void) {
    // Let's check the BnRobotIK_ArmZYY
    // The arms length are 0,1,1
    // For this type of test on Blender the Y is the python X axis
    // This is because Blender is forcing me to do this
    char const units[] = "cm";

    BnRobotIK_ArmZYY_t bnaik = BnRobotIK_ArmZYY_create(0, 1, 1, NULL, units);

    double const test_evalues1[3][3] = {{0, 0, NAN}, {0, 0, 0}, {0, 0, 0}};
    double test_ovalues[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double const test_endpoint1[3] = {0, 0, 2};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint1, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues2[3][3] = {{0, 0, 0}, {0, BnUtils_toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint2[3] = {2, 0, 0};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint2, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues3[3][3] = {{0, 0, 0}, {0, BnUtils_toRadians(45), 0}, {0, BnUtils_toRadians(45), 0}};
    double const test_endpoint3[3] = {1.711, 0.0, 0.703};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint3, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[2], test_ovalues[2], 3, 1e-2f, 1e-2f);

    double const test_evalues4[3][3] = {{0, 0, 0}, {0, BnUtils_toRadians(45), 0}, {0, 0, 0}};
    double const test_endpoint4[3] = {1.416, 0.0, 1.416};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint4, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues5[3][3] = {{0, 0, 0}, {0, BnUtils_toRadians(90), 0}, {0, BnUtils_toRadians(45), 0}};
    double const test_endpoint5[3] = {1.707, 0.0, -0.713};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint5, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues5[2], test_ovalues[2], 3, 1e-2f, 1e-2f);

    double const test_evalues6[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, BnUtils_toRadians(135), 0}};
    double const test_endpoint6[3] = {0.713, 0.0, 0.281};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint6, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues6[0], test_ovalues[0], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues6[1], test_ovalues[1], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues6[2], test_ovalues[2], 3, 1e-1f, 1e-1f);

    double const test_evalues7[3][3] = {{0, 0, NAN}, {0, 0, 0}, {0, BnUtils_toRadians(180), 0}};
    double const test_endpoint7[3] = {0.0, 0.0, 0.0};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint7, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues7[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues7[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues7[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues8[3][3] = {{0, 0, 0}, {0, BnUtils_toRadians(-60), 0}, {0, BnUtils_toRadians(150), 0}};
    double const test_endpoint8[3] = {0.1472482681274414, 0.0, 0.497156023979187};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint8, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues8[0], test_ovalues[0], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues8[1], test_ovalues[1], 3, 1e-1f, 1e-1f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues8[2], test_ovalues[2], 3, 1e-1f, 1e-1f);

    double const test_evalues9[3][3] = {{0, 0, BnUtils_toRadians(90)}, {0, BnUtils_toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint9[3] = {0, 2, 0};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint9, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues9[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues9[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues9[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues10[3][3] = {{0, 0, BnUtils_toRadians(-90)}, {0, BnUtils_toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint10[3] = {0, -2, 0};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint10, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues10[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues10[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues10[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues11[3][3] = {{0, 0, BnUtils_toRadians(180)}, {0, BnUtils_toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint11[3] = {-2, 0, 0};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint11, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues11[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues11[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues11[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const test_evalues12[3][3] = {
        {0, 0, BnUtils_toRadians(45)}, {0, BnUtils_toRadians(45), 0}, {0, BnUtils_toRadians(45), 0}};
    double const test_endpoint12[3] = {1.210, 1.210, 0.70};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint12, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues12[0], test_ovalues[0], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues12[1], test_ovalues[1], 3, 1e-2f, 1e-2f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues12[2], test_ovalues[2], 3, 1e-2f, 1e-2f);

    double const test_evalues13[3][3] = {{0, 0, BnUtils_toRadians(45)}, {0, BnUtils_toRadians(90), 0}, {0, 0, 0}};
    double const test_endpoint13[3] = {1.416094183921814, 1.416094183921814, 0.0};
    BnRobotIK_ArmZYY_compute(&bnaik, test_endpoint13, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues13[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues13[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues13[2], test_ovalues[2], 3, 1e-6f, 1e-6f);
}

void Test_BnRobotArm_MT(void) {

    // Arms along the Y axis
    double const initialPosition_1[3] = {0, 0, 0};
    double const armVector1_1[3] = {0, 1, 0};
    double const armVector2_1[3] = {0, 1, 0};
    char const units[] = "cm";

    BnMotionTracking_2Nodes_t bnmotiontrack =
        BnMotionTracking_2Nodes_create(initialPosition_1, armVector1_1, armVector2_1, NULL, units);

    BnRobotIK_ArmZYY_t bnaik = BnRobotIK_ArmZYY_create(0, 1, 1, NULL, units);

    BnRobotArm_MT_ArmZYY_2Nodes_t robotMT = BnRobotArm_MT_ArmZYY_2Nodes_create(&bnmotiontrack, &bnaik);

    // X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    double endpos[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double const node1_quat1[4] = {0.92388, 0.382683, 0, 0};
    double const node2_quat1[4] = {1, 0, 0, 0};

    // [90.         45.00005363 44.99993373]]
    double const test_evalues1[3][3] = {
        {0, 0, BnUtils_toRadians(90)}, {0, BnUtils_toRadians(45.00005363), 0}, {0, BnUtils_toRadians(44.99993373), 0}};
    double test_ovalues[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    BnRobotArm_MT_ArmZYY_2Nodes_compute(&robotMT, node1_quat1, node2_quat1, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    // ---------
    // Arms along the X axis
    double const initialPosition_2[3] = {0, 0, 0};
    double const armVector1_2[3] = {1, 0, 0};
    double const armVector2_2[3] = {1, 0, 0};

    bnmotiontrack = BnMotionTracking_2Nodes_create(initialPosition_2, armVector1_2, armVector2_2, NULL, units);

    bnaik = BnRobotIK_ArmZYY_create(0, 1, 1, NULL, units);

    // Y rotation 80, -20 -> [ 1.12019681930542, 0.0, 0.634331464767456 ]
    double const node1_quat2[4] = {0.766044, 0, -0.642788, 0};
    double const node2_quat2[4] = {0.984808, 0, 0.173648, 0};

    robotMT = BnRobotArm_MT_ArmZYY_2Nodes_create(&bnmotiontrack, &bnaik);

    // [0.         10. 100.]]
    double const test_evalues2[3][3] = {
        {0, 0, 0}, {0, BnUtils_toRadians(9.99994606), 0}, {0, BnUtils_toRadians(100.00004607), 0}};

    BnRobotArm_MT_ArmZYY_2Nodes_compute(&robotMT, node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[2], test_ovalues[2], 3, 1e-6f, 1e-6f);
}

void Test_BnRobotArm_MT_Constraints(void) {

    // Arms along the Y axis
    double const initialPosition_1[3] = {0, 0, 0};
    double const armVector1_1[3] = {0, 1, 0};
    double const armVector2_1[3] = {0, 1, 0};
    char const units[] = "cm";

    BnMotionTracking_2Nodes_t bnmotiontrack =
        BnMotionTracking_2Nodes_create(initialPosition_1, armVector1_1, armVector2_1, NULL, units);

    double const anglesConstraints1[3][2] = {
        {BnUtils_toRadians(-45), BnUtils_toRadians(45)}, {0, BnUtils_toRadians(90)}, {0, BnUtils_toRadians(90)}};
    BnRobotIK_ArmZYY_t bnaik = BnRobotIK_ArmZYY_create(0, 1, 1, anglesConstraints1, units);

    BnRobotArm_MT_ArmZYY_2Nodes_t robotMT = BnRobotArm_MT_ArmZYY_2Nodes_create(&bnmotiontrack, &bnaik);

    // X rotation 45, 0 -> [ 0.0, 1.711, 0.703 ]
    double endpos[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double const node1_quat1[4] = {0.92388, 0.382683, 0, 0};
    double const node2_quat1[4] = {1, 0, 0, 0};

    // [45.         45.00005363 44.99993373]]
    double const test_evalues1[3][3] = {
        {0, 0, BnUtils_toRadians(45)}, {0, BnUtils_toRadians(45.00005363), 0}, {0, BnUtils_toRadians(44.99993373), 0}};
    double test_ovalues[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    BnRobotArm_MT_ArmZYY_2Nodes_compute(&robotMT, node1_quat1, node2_quat1, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues1[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    // ---------
    // Arms along the X axis
    double const initialPosition_2[3] = {0, 0, 0};
    double const armVector1_2[3] = {1, 0, 0};
    double const armVector2_2[3] = {1, 0, 0};

    bnmotiontrack = BnMotionTracking_2Nodes_create(initialPosition_2, armVector1_2, armVector2_2, NULL, units);

    double const anglesConstraints2[3][2] = {
        {BnUtils_toRadians(-90), BnUtils_toRadians(90)}, {0, BnUtils_toRadians(90)}, {0, BnUtils_toRadians(90)}};
    bnaik = BnRobotIK_ArmZYY_create(0, 1, 1, anglesConstraints2, units);

    robotMT = BnRobotArm_MT_ArmZYY_2Nodes_create(&bnmotiontrack, &bnaik);

    // Z rotation 135
    // Y rotation 10, 20 -> [ -1.3624130487442017, 1.3624131679534912,
    // -0.5175356268882751 ]
    double const node1_quat2[4] = {0.381227, -0.080521, 0.033353, 0.920364};
    double const node2_quat2[4] = {0.37687, -0.16043, 0.066452, 0.909844};

    // [90.         90 30]]
    double const test_evalues2[3][3] = {
        {0, 0, BnUtils_toRadians(90)}, {0, BnUtils_toRadians(90), 0}, {0, BnUtils_toRadians(29.15184909), 0}};

    BnRobotArm_MT_ArmZYY_2Nodes_compute(&robotMT, node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues2[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const anglesConstraints3[3][2] = {
        {BnUtils_toRadians(-90), BnUtils_toRadians(90)}, {0, BnUtils_toRadians(100)}, {0, BnUtils_toRadians(90)}};
    bnaik = BnRobotIK_ArmZYY_create(0, 1, 1, anglesConstraints3, units);

    robotMT = BnRobotArm_MT_ArmZYY_2Nodes_create(&bnmotiontrack, &bnaik);

    // [90.         100 10]]
    double const test_evalues3[3][3] = {
        {0, 0, BnUtils_toRadians(90)}, {0, BnUtils_toRadians(100), 0}, {0, BnUtils_toRadians(9.9999254), 0}};

    BnRobotArm_MT_ArmZYY_2Nodes_compute(&robotMT, node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues3[2], test_ovalues[2], 3, 1e-6f, 1e-6f);

    double const anglesConstraints4[3][2] = {
        {BnUtils_toRadians(-90), BnUtils_toRadians(90)}, {0, BnUtils_toRadians(180)}, {0, BnUtils_toRadians(90)}};
    bnaik = BnRobotIK_ArmZYY_create(0, 1, 1, anglesConstraints4, units);

    robotMT = BnRobotArm_MT_ArmZYY_2Nodes_create(&bnmotiontrack, &bnaik);

    // [90.         100 10]]
    double const test_evalues4[3][3] = {
        {0, 0, BnUtils_toRadians(90)}, {0, BnUtils_toRadians(100.00035347), 0}, {0, BnUtils_toRadians(9.99922384), 0}};

    BnRobotArm_MT_ArmZYY_2Nodes_compute(&robotMT, node1_quat2, node2_quat2, endpos, test_ovalues);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[0], test_ovalues[0], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[1], test_ovalues[1], 3, 1e-6f, 1e-6f);
    TEST_ASSERT_DOUBLE_ARRAY(test_evalues4[2], test_ovalues[2], 3, 1e-6f, 1e-6f);
}

///////////////////

void setUp(void) {
    // This runs before each test. Leave empty if not needed.
}

void tearDown(void) {
    // This runs after each test. Leave empty if not needed.
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(Test_BnConstants);
    RUN_TEST(Test_BnQuaternion);
    RUN_TEST(Test_BnUtils);
    RUN_TEST(Test_BnAxisConfig);
    RUN_TEST(Test_BnMotionTracking_2Nodes);
    RUN_TEST(Test_BnMotionTracking_2NodesConstraint);
    RUN_TEST(Test_BnRobotIK_ArmZYY);
    RUN_TEST(Test_BlenderSimpleLinksProj1);
    RUN_TEST(Test_BlenderSimpleLinksProj2);
    RUN_TEST(Test_BlenderSimpleLinksProj3);
    RUN_TEST(Test_BnRobotArm_MT);
    RUN_TEST(Test_BnRobotArm_MT_Constraints);

    return UNITY_END();
}