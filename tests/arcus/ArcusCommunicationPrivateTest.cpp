//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunicationPrivateTest.h"

#include "../src/Application.h"
#include "../src/Slice.h"

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(ArcusCommunicationPrivateTest);

constexpr size_t gkTestNumMeshGroups = 1;

void ArcusCommunicationPrivateTest::setUp()
{
    instance = new ArcusCommunication::Private();

    Application::getInstance().current_slice = new Slice(gkTestNumMeshGroups);
}

void ArcusCommunicationPrivateTest::tearDown()
{
    delete instance;

    delete Application::getInstance().current_slice;
}

void ArcusCommunicationPrivateTest::readGlobalSettingsMessageTest()
{
    // Read 'real-life' global settings from file:
    std::ifstream test_settings_file("../tests/test_global_settings.txt");
    CPPUNIT_ASSERT(test_settings_file.is_open());

    cura::proto::SettingList global_settings;
    std::unordered_map<std::string, std::string> raw_settings;

    std::string line;
    while (std::getline(test_settings_file, line))
    {
        size_t pos = line.find_first_of(',');
        if (line.size() < 3 || pos == std::string::npos) // <<- Whitespace, etc.
        {
            continue;
        }
        
        const std::string key = line.substr(0, pos);
        const std::string value = line.substr(pos + 1, std::string::npos);

        raw_settings.insert({key, value});

        cura::proto::Setting* entry = global_settings.add_settings();
        entry->set_name(key);
        entry->set_value(value);
    }
    test_settings_file.close();
    CPPUNIT_ASSERT(! raw_settings.empty());
    CPPUNIT_ASSERT_EQUAL(static_cast<size_t>(global_settings.settings_size()), raw_settings.size());
    
    // The call it's actually all about:
    instance->readGlobalSettingsMessage(global_settings);
    
    // Check if they are equal in general:
    const auto& settings = Application::getInstance().current_slice->scene.settings;
    for (const auto& entry : raw_settings)
    {
        CPPUNIT_ASSERT_EQUAL(settings.get<std::string>(entry.first), entry.second);
    }
}

void ArcusCommunicationPrivateTest::readExtruderSettingsMessageTest()
{

}

} //namespace cura