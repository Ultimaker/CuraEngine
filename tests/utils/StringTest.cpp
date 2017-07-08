//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "StringTest.h"

#include <iomanip>
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
void StringTest::writeInt2mmTestMax()
{
    writeInt2mmAssert(std::numeric_limits<int32_t>::max() / 1001); // divide by 1001, because MM2INT first converts to int and then multiplies by 1000, which causes overflow for the highest integer.
}


void StringTest::writeInt2mmAssert(int64_t in)
{
    std::ostringstream ss;
    writeInt2mm(in, ss);

    ss.flush();
    std::string str = ss.str();
    if (!ss.good())
    {
        char buffer[200];
        sprintf(buffer, "The integer %ld was printed as '%s' which was a bad string!", in, str.c_str());
        CPPUNIT_ASSERT_MESSAGE(std::string(buffer), false);
    }
    int64_t out = MM2INT(strtod(str.c_str(), nullptr));

    char buffer[200];
    sprintf(buffer, "The integer %ld was printed as '%s' which was interpreted as %ld rather than %ld!", in, str.c_str(), out, in);
    CPPUNIT_ASSERT_MESSAGE(std::string(buffer), in == out);
}


void StringTest::writeDoubleToStreamTest10000Negative()
{
    writeDoubleToStreamAssert(-10.000);
}
void StringTest::writeDoubleToStreamTest1000Negative()
{
    writeDoubleToStreamAssert(-1.000);
}
void StringTest::writeDoubleToStreamTest100Negative()
{
    writeDoubleToStreamAssert(-.100);
}
void StringTest::writeDoubleToStreamTest10Negative()
{
    writeDoubleToStreamAssert(-.010);
}
void StringTest::writeDoubleToStreamTest1Negative()
{
    writeDoubleToStreamAssert(-.001);
}
void StringTest::writeDoubleToStreamTest0()
{
    writeDoubleToStreamAssert(0.000);
}
void StringTest::writeDoubleToStreamTest1()
{
    writeDoubleToStreamAssert(.001);
}
void StringTest::writeDoubleToStreamTest10()
{
    writeDoubleToStreamAssert(.010);
}
void StringTest::writeDoubleToStreamTest100()
{
    writeDoubleToStreamAssert(.100);
}
void StringTest::writeDoubleToStreamTest1000()
{
    writeDoubleToStreamAssert(1.000);
}
void StringTest::writeDoubleToStreamTest10000()
{
    writeDoubleToStreamAssert(10.000);
}
void StringTest::writeDoubleToStreamTest123456789()
{
    writeDoubleToStreamAssert(123456.789);
}


void StringTest::writeDoubleToStreamTestMin()
{
    writeDoubleToStreamAssert(std::numeric_limits<double>::min());
}
void StringTest::writeDoubleToStreamTestMax()
{
    writeDoubleToStreamAssert(std::numeric_limits<double>::max());
}
void StringTest::writeDoubleToStreamTestLowest()
{
    writeDoubleToStreamAssert(std::numeric_limits<double>::lowest());
}
void StringTest::writeDoubleToStreamTestLowestNeg()
{
    writeDoubleToStreamAssert(-std::numeric_limits<double>::lowest());
}
void StringTest::writeDoubleToStreamTestLow()
{
    writeDoubleToStreamAssert(0.00000001d);
}


void StringTest::writeDoubleToStreamAssert(double in, unsigned int precision)
{
    std::ostringstream ss;
    writeDoubleToStream(precision, in, ss);
    ss.flush();
    std::string str = ss.str();
    if (!ss.good())
    {
        char buffer[8000];
        sprintf(buffer, "The double %f was printed as '%s' which was a bad string!", in, str.c_str());
        CPPUNIT_ASSERT_MESSAGE(std::string(buffer), false);
    }
    double out = strtod(str.c_str(), nullptr);

    std::ostringstream in_ss;
    in_ss << std::fixed << std::setprecision(precision) << in;
    std::string in_str = in_ss.str();
    double in_reinterpreted = strtod(in_str.c_str(), nullptr);
    
    char buffer[8000];
    sprintf(buffer, "The double %f was printed as '%s' which was interpreted as %f rather than %f!", in, str.c_str(), out, in_reinterpreted);
    if (in_reinterpreted != out) std::cerr << buffer << "\n";
    CPPUNIT_ASSERT_MESSAGE(std::string(buffer), in_reinterpreted == out);
}


}