//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "StringTest.h"

#include <sstream> // ostringstream
#include <../src/utils/intpoint.h>
#include <../src/utils/string.h>

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(StringTest);

void StringTest::setUp()
{
    //Do nothing.
}

void StringTest::tearDown()
{
    //Do nothing.
}

void StringTest::writeInt2mmTest10000Negative()
{
    writeInt2mmAssert(-10000);
}
void StringTest::writeInt2mmTest1000Negative()
{
    writeInt2mmAssert(-1000);
}
void StringTest::writeInt2mmTest100Negative()
{
    writeInt2mmAssert(-100);
}
void StringTest::writeInt2mmTest10Negative()
{
    writeInt2mmAssert(-10);
}
void StringTest::writeInt2mmTest1Negative()
{
    writeInt2mmAssert(-1);
}
void StringTest::writeInt2mmTest0()
{
    writeInt2mmAssert(0);
}
void StringTest::writeInt2mmTest1()
{
    writeInt2mmAssert(1);
}
void StringTest::writeInt2mmTest10()
{
    writeInt2mmAssert(10);
}
void StringTest::writeInt2mmTest100()
{
    writeInt2mmAssert(100);
}
void StringTest::writeInt2mmTest1000()
{
    writeInt2mmAssert(1000);
}
void StringTest::writeInt2mmTest10000()
{
    writeInt2mmAssert(10000);
}
void StringTest::writeInt2mmTest123456789()
{
    writeInt2mmAssert(123456789);
}


void StringTest::writeInt2mmAssert(int64_t in)
{
    std::ostringstream ss;
    writeInt2mm(in, ss);
    std::string str = ss.str();
    int64_t out = MM2INT(strtod(str.c_str(), nullptr));

    char buffer[200];
    sprintf(buffer, "The integer %ld was printed as '%s' which was interpreted as %ld rather than %ld!", in, str.c_str(), out, in);
    CPPUNIT_ASSERT_MESSAGE(std::string(buffer), in == out);
}


}