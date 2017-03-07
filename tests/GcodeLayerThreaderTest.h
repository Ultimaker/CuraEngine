//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GCODE_LAYER_THREADER_TEST_H
#define GCODE_LAYER_THREADER_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace cura
{

class GcodeLayerThreaderTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(GcodeLayerThreaderTest);
    CPPUNIT_TEST(test1);
    CPPUNIT_TEST(test2);
    CPPUNIT_TEST(test3);
    CPPUNIT_TEST(test4);

    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * Since <em>StringTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * Since <em>StringTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();

    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void test1();
    void test2();
    void test3();
    void test4();
private:
    void test(int avg_computation_time, int layer_count, int max_task_count);
    class LayerPlan
    {
    public:
        LayerPlan(int layer_nr)
        : layer_nr(layer_nr)
        {
        }
        int layer_nr;
        std::vector<int> buffered_items;
    };
};

}

#endif //GCODE_LAYER_THREADER_TEST_H