//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunicationPrivateTest.h"

namespace cura
{

void ArcusCommunicationPrivateTest::setUp()
{
    instance = new ArcusCommunication::Private();
}

void ArcusCommunicationPrivateTest::tearDown()
{
    delete instance;
}

void ArcusCommunicationPrivateTest::readExtruderSettingsMessageTest()
{
    
}

} //namespace cura