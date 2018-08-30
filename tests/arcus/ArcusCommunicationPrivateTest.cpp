//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunicationPrivateTest.h"
#include "MockSocket.h"

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(ArcusCommunicationPrivateTest);

void ArcusCommunicationPrivateTest::setUp()
{
    instance = new ArcusCommunication::Private();
    instance->socket = new MockSocket();
}

void ArcusCommunicationPrivateTest::tearDown()
{
    delete instance->socket;
    delete instance;
}

void ArcusCommunicationPrivateTest::readExtruderSettingsMessageTest()
{
    
}

} //namespace cura