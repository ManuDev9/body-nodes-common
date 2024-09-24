#include "BnTestRegistry.hpp"
#include "BnReorientAxis.h"
#include "BnTwoNodesMotionTracking.h"
#include "BnRobotIK_ZYY2Arms.h"

/**
 * Checks if two arrays are close to each other within given absolute and relative tolerances.
 * 
 * @param actual Array of actual values.
 * @param expected Array of expected values.
 * @param length Lenght of both arrays
 * @param absError Absolute error tolerance.
 * @param relError Relative error tolerance.
 * @return True if arrays are close within tolerances, false otherwise.
 */
static bool areArraysClose(float expected[], float actual[], uint64_t length, float absError, float relError) {

    for (int i = 0; i < length; i++) {
        float diffTmp = actual[i] - expected[i];
        float diff = diffTmp > 0 ? diffTmp : -diffTmp;
        float expectedValue = expected[i] > 0 ? expected[i] : -expected[i];

        // Check if the difference is within absolute or relative error bounds
        if (diff > absError && (expectedValue == 0 || diff / expectedValue > relError)) {
            return false;
        }
    }

    return true;
}

TEST_CASE(BnReorientAxisTest) {
    
    int test_io_axis[] = { 3, 2, 1, 0 };
    int test_io_sign[] = { -1, -1, -1, -1 };
    float test_ivalues[] = { 3.0f, 8.0f, 2.0f, 10.2f };
    float test_ovalues[] = { 3.0f, 8.0f, 2.0f, 10.2f };
    float test_evalues[] = { -10.2f, -2.0f, -8.0f, -3.0f };

    bodynodesdev::common::BnReorientAxis test_obj;
    test_obj.config( test_io_axis, test_io_sign, 4);
    test_obj.apply( test_ovalues );

    ASSERT(areArraysClose(test_evalues, test_ovalues, 4, 1e-5f, 1e-3f ));
}

TEST_CASE(BnTwoNodesMotionTrackingTest) {
    
    float test_node1_quat[] = { 0.9926f, 0.0329f, 0.0973f, 0.0640f };
    float test_node2_quat[] = { 0.9583f, -0.1367f, -0.0595f, -0.2439f };
    float test_ovalues[] = { 0,0,0 };
    float test_evalues[] = { 18.468636171839087f, -3.1761790635934757f, -0.08354223767877755f };

    float initialPosition[3] = { 0,0,0 };
    float lengthArm1 = 10;
    float lengthArm2 = 10;
    float locationConstraints[6] = { 10, 20 , -5, 5,  -5, 5 };
    std::string units = "cm";
            
    bodynodesdev::common::BnTwoNodesMotionTracking test_obj(
        initialPosition, lengthArm1, lengthArm2, locationConstraints, units
    );
    test_obj.compute( test_node1_quat, test_node2_quat, test_ovalues);

    ASSERT(areArraysClose(test_evalues, test_ovalues, 3, 1e-2f, 1e-2f ));
}

TEST_CASE(BnTwoNodesMotionTrackingConstraintTest) {
    
    float test_node1_quat[] = { 0.8504f, 0.3678f, -0.1840f, 0.3281f };
    float test_node2_quat[] = { 0.9293f, -0.0039f, -0.2892f, 0.2296f };
    float test_ovalues[] = { 0,0,0 };
    float test_evalues[] = { 14.443218483410508f, 5, 5 };

    float initialPosition[3] = { 0,0,0 };
    float lengthArm1 = 10;
    float lengthArm2 = 10;
    float locationConstraints[6] = { 10, 20 , -5, 5,  -5, 5 };
    std::string units = "cm";
            
    bodynodesdev::common::BnTwoNodesMotionTracking test_obj(
        initialPosition, lengthArm1, lengthArm2, locationConstraints, units
    );
    test_obj.compute( test_node1_quat, test_node2_quat, test_ovalues);

    ASSERT(areArraysClose(test_evalues, test_ovalues, 3, 1e-5f, 1e-3f ));
}

TEST_CASE(BnRobotIK_ZYY2ArmsTest) {
    
    float test_endpoint[] = { 18.219124272891392f, 3.8972461548699857f, 1.6501078154541111f };
    float test_ovalues[] = { 0,0,0 };
    float test_evalues[] = { 0.21073373345528476f, -0.4522653965641126f, 0.723883473845901f };

    float lengthRA2 = 10;
    float lengthRA3 = 10;
    float displSP[3] = { 0,0,0 };
    std::string units = "cm";

    bodynodesdev::common::BnRobotIK_ZYY2Arms test_obj(
        lengthRA2, lengthRA3, displSP, units
    );
    test_obj.compute( test_endpoint, test_ovalues);

    ASSERT(areArraysClose(test_evalues, test_ovalues, 3, 1e-5f, 1e-3f ));
}

int main(){
    BnTestRegistry::getTestRegistry().runAllTests();
    return 0;
}




