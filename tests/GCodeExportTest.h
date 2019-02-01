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
    CPPUNIT_TEST(commentMultiLine);
    CPPUNIT_TEST(commentMultiple);
    CPPUNIT_TEST(commentTimeZero);
    CPPUNIT_TEST(commentTimeInteger);
    CPPUNIT_TEST(commentTimeFloatRoundingError);
    CPPUNIT_TEST(commentTypeAllTypesCovered);
    CPPUNIT_TEST(commentLayer);
    CPPUNIT_TEST_SUITE_END();

public:
    /*
     * Resets the fixtures for a new test.
     */
    void setUp();

    /*
     * Tests for writing comments.
     */
    void commentEmpty(); //Empty message.
    void commentSimple(); //Just a normal message.
    void commentMultiLine(); //Contains newlines.
    void commentMultiple(); //Multiple comments in a row.
    void commentTimeZero(); //Simplest printing time.
    void commentTimeInteger(); //Printing a simple number of seconds.
    void commentTimeFloatRoundingError(); //Tests hardness against rounding errors in time estimates.
    void commentTypeAllTypesCovered(); //Tests if all feature types can be printed.
    void commentLayer(); //Printing layer numbers.

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