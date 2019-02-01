//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GCODEEXPORTTEST_H
#define GCODEEXPORTTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/gcodeExport.h" //The class we're testing.

namespace cura
{

/*
 * Tests whether the g-code export stage produces g-code that does what we want.
 */
class GCodeExportTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(GCodeExportTest);
    CPPUNIT_TEST(commentEmpty);
    CPPUNIT_TEST(commentSimple);
    CPPUNIT_TEST_SUITE_END();

public:
    /*
     * Resets the fixtures for a new test.
     */
    void setUp();

    /*
     * Tests for writing comments.
     */
    void commentEmpty();
    void commentSimple();

private:
    /*
     * An export class to test with.
     */
    GCodeExport gcode;

    /*
     * A stream to capture the output of the g-code export.
     */
    std::stringstream output;
};

} //namespace cura

#endif //GCODEEXPORTTEST_H