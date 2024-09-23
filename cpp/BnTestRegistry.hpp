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

#define ASSERT(val) \
    do { \
        if (!(val)) { \
            std::cout << __FILE__ << ":" << __LINE__ << ": Test failed." << std::endl; \
            return; \
        } \
        else { \
          std::cout << __FILE__ << ":" << __LINE__ << ": Test Passed." << std::endl; \
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
