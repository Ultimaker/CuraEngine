//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ArcusCommunicationPrivateTest.h"
#include "MockSocket.h"

#include "../src/Application.h"
#include "../src/Slice.h"

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(ArcusCommunicationPrivateTest);

constexpr size_t gkTestNumMeshGroups = 1;

void ArcusCommunicationPrivateTest::setUp()
{
    instance = new ArcusCommunication::Private();
    instance->socket = new MockSocket();
    Application::getInstance().current_slice = new Slice(gkTestNumMeshGroups);
}

void ArcusCommunicationPrivateTest::tearDown()
{
    delete instance->socket;
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

void ArcusCommunicationPrivateTest::readSingleExtruderSettingsMessageTest()
{
    google::protobuf::RepeatedPtrField<proto::Extruder> messages; //Construct a message.

    //Test with one extruder.
    proto::Extruder* extruder_message = messages.Add();
    extruder_message->set_id(0);

    //Fill the extruder with settings.
    proto::SettingList* extruder_settings = extruder_message->mutable_settings();
    proto::Setting* setting = extruder_settings->add_settings();
    setting->set_name("test_setting");
    const std::string setting_value = "You put the 'sexy' in 'dyslexic'.";
    setting->set_value(setting_value);

    Application::getInstance().current_slice->scene.settings.add("machine_extruder_count", "1");
    //Run the call that we're testing.
    instance->readExtruderSettingsMessage(messages);

    CPPUNIT_ASSERT_EQUAL_MESSAGE("Reading the extruders must construct the correct amount of extruders in the scene.",
                                 size_t(1), Application::getInstance().current_slice->scene.extruders.size());
    CPPUNIT_ASSERT_EQUAL(setting_value, Application::getInstance().current_slice->scene.extruders[0].settings.get<std::string>("test_setting"));
}

void ArcusCommunicationPrivateTest::readMultiExtruderSettingsMessageTest()
{
    google::protobuf::RepeatedPtrField<proto::Extruder> messages; //Construct a message.

    //Test with two extruders.
    proto::Extruder* second_extruder = messages.Add(); //Out of order on purpose.
    second_extruder->set_id(1);
    proto::Extruder* first_extruder = messages.Add();
    first_extruder->set_id(0);

    //Give a different value for each extruder.
    proto::SettingList* first_extruder_settings = first_extruder->mutable_settings();
    proto::Setting* first_setting = first_extruder_settings->add_settings();
    first_setting->set_name("What extruder are you?");
    first_setting->set_value("First");
    proto::SettingList* second_extruder_settings = second_extruder->mutable_settings();
    proto::Setting* second_setting = second_extruder_settings->add_settings();
    second_setting->set_name("What extruder are you?");
    second_setting->set_value("Second");

    Application::getInstance().current_slice->scene.settings.add("machine_extruder_count", "2");
    //Run the call that we're testing.
    instance->readExtruderSettingsMessage(messages);

    CPPUNIT_ASSERT_EQUAL_MESSAGE("Reading the extruders must construct the correct amount of extruders in the scene.",
                                 size_t(2), Application::getInstance().current_slice->scene.extruders.size());
    CPPUNIT_ASSERT_EQUAL(std::string("First"), Application::getInstance().current_slice->scene.extruders[0].settings.get<std::string>("What extruder are you?"));
    CPPUNIT_ASSERT_EQUAL(std::string("Second"), Application::getInstance().current_slice->scene.extruders[1].settings.get<std::string>("What extruder are you?"));
}

} //namespace cura