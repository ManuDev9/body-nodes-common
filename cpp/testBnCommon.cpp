#include "BnTestRegistry.hpp"
#include "BnReorientAxis.h"


TEST_CASE(BnReorientAxisObj) {
    
    int test_io_axis[] = { 3, 2, 1, 0 };
    int test_io_sign[] = { -1, -1, -1, -1 };
    float test_ivalues[] = { 3.0f, 8.0f, 2.0f, 10.2f };
    float test_ovalues[] = { 3.0f, 8.0f, 2.0f, 10.2f };
    float test_evalues[] = { -10.2f, -2.0f, -8.0f, -3.0f };

    BnReorientAxis test_obj;
    test_obj.config( test_io_axis, test_io_sign, 4);
    test_obj.apply( test_ovalues );

    ASSERT_EQUAL(test_evalues[0], test_ovalues[0]);
    ASSERT_EQUAL(test_evalues[1], test_ovalues[1]);
    ASSERT_EQUAL(test_evalues[2], test_ovalues[2]);
    ASSERT_EQUAL(test_evalues[3], test_ovalues[3]);
}


int main(){
    BnTestRegistry::getTestRegistry().runAllTests();
    return 0;
}




