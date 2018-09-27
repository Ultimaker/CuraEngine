//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ARCUSCOMMUNICATIONPRIVATETEST_H
#define ARCUSCOMMUNICATIONPRIVATETEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <memory>

#include "../../src/communication/ArcusCommunicationPrivate.h" //The class we're testing.

namespace cura
{

class ArcusCommunicationPrivateTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(ArcusCommunicationPrivateTest);

    CPPUNIT_TEST(readGlobalSettingsMessageTest);
    CPPUNIT_TEST(readSingleExtruderSettingsMessageTest);
    CPPUNIT_TEST(readMultiExtruderSettingsMessageTest);
    CPPUNIT_TEST(readMeshGroupMessageTest);

    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();
    void tearDown();

    //Our unit tests.
    void readGlobalSettingsMessageTest();
    void readSingleExtruderSettingsMessageTest();
    void readMultiExtruderSettingsMessageTest();
    void readMeshGroupMessageTest();

    ArcusCommunication::Private* instance;
};

} //namespace cura

#endif //ARCUSCOMMUNICATIONPRIVATETEST_H