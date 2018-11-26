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

// Settings need to be loaded into different protobuf-objects during different tests.
template<typename T>
void loadTestSettings(const std::string& filename, T* p_settings, std::unordered_map<std::string, std::string>* p_raw_settings)
{
    T& settings = *p_settings;
    std::unordered_map<std::string, std::string>& raw_settings = *p_raw_settings;

    std::ifstream test_settings_file(filename);
    CPPUNIT_ASSERT(test_settings_file.is_open());

    std::string line;
    while (std::getline(test_settings_file, line))
    {
        size_t pos = line.find_first_of('=');
        if (line.size() < 3 || pos == std::string::npos) // <<- Whitespace, etc.
        {
            continue;
        }
        
        const std::string key = line.substr(0, pos);
        const std::string value = line.substr(pos + 1, std::string::npos);

        raw_settings.insert({key, value});

        cura::proto::Setting* entry = settings.add_settings();
        entry->set_name(key);
        entry->set_value(value);
    }
    test_settings_file.close();
    CPPUNIT_ASSERT(!raw_settings.empty());
    CPPUNIT_ASSERT_EQUAL(static_cast<size_t>(settings.settings_size()), raw_settings.size());
}

void ArcusCommunicationPrivateTest::readGlobalSettingsMessageTest()
{
    // Read 'real-life' global settings from file:

    cura::proto::SettingList global_settings;
    std::unordered_map<std::string, std::string> raw_settings;
    loadTestSettings("../tests/test_global_settings.txt", &global_settings, &raw_settings);

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

void ArcusCommunicationPrivateTest::readMeshGroupMessageTest()
{
    // Setup:
    cura::proto::ObjectList mesh_message;

    // - Load 'global' settings:
    std::unordered_map<std::string, std::string> raw_settings;
    loadTestSettings("../tests/test_global_settings.txt", &mesh_message, &raw_settings);

    // - Create mesh-message-mesh:
    cura::proto::Object* mesh = mesh_message.add_objects();

    // - - Read cube vertices from a test-file, then add to message:
    std::ifstream cube_verts_file("../tests/cube_vertices.txt");
    CPPUNIT_ASSERT(cube_verts_file.is_open());

    std::vector<float> raw_vertices;

    float next;
    while (cube_verts_file >> next)
    {
        raw_vertices.push_back(next);
    }
    cube_verts_file.close();
    CPPUNIT_ASSERT(raw_vertices.size() % 3 == 0 && ! raw_vertices.empty());

    // (NOTE: *Don't* replace the below by strncopy, direct call to constructor, etc. in any way. We need to pass '/0' inside the string. Blame protobuf!)
    const size_t num_str = sizeof(float) * raw_vertices.size();
    uint8_t* data = reinterpret_cast<uint8_t*>(raw_vertices.data());
    std::string verts_as_str;
    verts_as_str.assign(num_str, ' ');
    for (size_t i_char = 0; i_char < num_str; ++i_char)
    {
        verts_as_str[i_char] = data[i_char];
    }
    mesh->set_vertices(verts_as_str);

    // - - Add settings to the mesh:
    cura::proto::Setting* entry = mesh->add_settings();
    entry->set_name("extruder_nr");
    entry->set_value("0");
    entry = mesh->add_settings();
    entry->set_name("center_object");
    entry->set_value("1");
    entry = mesh->add_settings();
    entry->set_name("mesh_position_x");
    entry->set_value("0");
    entry = mesh->add_settings();
    entry->set_name("mesh_position_y");
    entry->set_value("0");
    entry = mesh->add_settings();
    entry->set_name("mesh_position_z");
    entry->set_value("0");
    entry = mesh->add_settings();

    // The call it's actually all about:
    instance->readMeshGroupMessage(mesh_message);

    // Checks:
    auto& scene = Application::getInstance().current_slice->scene;
    CPPUNIT_ASSERT(!scene.mesh_groups.empty());

    auto& meshes = scene.mesh_groups[0].meshes;
    CPPUNIT_ASSERT(!meshes.empty());

    auto& vertices = meshes[0].vertices;
    CPPUNIT_ASSERT(!vertices.empty());
    CPPUNIT_ASSERT_EQUAL(vertices.size(), size_t(8)); //A cube should have 8 unique vertices.
    CPPUNIT_ASSERT_EQUAL(meshes[0].faces.size(), size_t(12)); // A cube should have 12 tri-s (2 for each 6 sides of the dice).

    // Distances should be the same:

    // - First, collect AABBoxes:
    std::array<coord_t, 3> raw_min_coords = {std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max()};
    std::array<coord_t, 3> raw_max_coords = {std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min()};
    const size_t num_vertex = raw_vertices.size();
    for (size_t i_coord = 0; i_coord < num_vertex; ++i_coord)
    {
        coord_t micrometers = static_cast<coord_t>(raw_vertices[i_coord] * 1000.f);
        raw_min_coords[i_coord % 3] = std::min(micrometers, raw_min_coords[i_coord % 3]);
        raw_max_coords[i_coord % 3] = std::max(micrometers, raw_max_coords[i_coord % 3]);
    }

    std::array<coord_t, 3> min_coords = {std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max(), std::numeric_limits<coord_t>::max()};
    std::array<coord_t, 3> max_coords = {std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min(), std::numeric_limits<coord_t>::min()};
    for (const auto& vertex : vertices)
    {
        min_coords[0] = std::min(vertex.p.x, min_coords[0]);
        min_coords[1] = std::min(vertex.p.y, min_coords[1]);
        min_coords[2] = std::min(vertex.p.z, min_coords[2]);
        max_coords[0] = std::max(vertex.p.x, max_coords[0]);
        max_coords[1] = std::max(vertex.p.y, max_coords[1]);
        max_coords[2] = std::max(vertex.p.z, max_coords[2]);
    }

    // - Then, just compare:
    for (int i = 0; i < 3; ++i)
    {
        CPPUNIT_ASSERT_EQUAL(max_coords[i] - min_coords[i], raw_max_coords[i] - raw_min_coords[i]);
    }
}

} //namespace cura