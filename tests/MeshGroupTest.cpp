// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "MeshGroup.h" // The unit under test: loadMeshIntoMeshGroup -> binary STL loader.

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>

#include "Application.h" // To set up a slice so that settings can be requested.
#include "Slice.h"
#include "settings/Settings.h"
#include "utils/Matrix4x3D.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

class MeshGroupTest : public testing::Test
{
public:
    void SetUp() override
    {
        Application::getInstance().startThreadPool();
        Application::getInstance().current_slice_ = std::make_shared<Slice>(1);
    }

    static void appendFloat(std::vector<uint8_t>& bytes, const float value)
    {
        std::array<uint8_t, sizeof(float)> raw{};
        std::memcpy(raw.data(), &value, sizeof(float));
        bytes.insert(bytes.end(), raw.begin(), raw.end());
    }

    //! Build a minimal binary STL: 80-byte header, \p reported_count as the uint32 triangle count, then
    //! one 50-byte record per triangle (each triangle is 9 vertex floats; the normal is zeroed).
    static std::vector<uint8_t> makeBinaryStl(const uint32_t reported_count, const std::vector<std::array<float, 9>>& triangles)
    {
        std::vector<uint8_t> bytes(80, 0x00);
        std::array<uint8_t, sizeof(uint32_t)> count_raw{};
        std::memcpy(count_raw.data(), &reported_count, sizeof(uint32_t));
        bytes.insert(bytes.end(), count_raw.begin(), count_raw.end());
        for (const std::array<float, 9>& triangle : triangles)
        {
            for (int normal = 0; normal < 3; ++normal)
            {
                appendFloat(bytes, 0.0F);
            }
            for (const float coord : triangle)
            {
                appendFloat(bytes, coord);
            }
            bytes.push_back(0); // 2-byte attribute
            bytes.push_back(0);
        }
        return bytes;
    }

    static std::string writeTempFile(const std::string& name, const std::vector<uint8_t>& bytes)
    {
        const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        out.close();
        return path.string();
    }
};

// Regression: a binary STL smaller than the 84-byte header must be rejected instead of underflowing the
// triangle-count arithmetic into a gigantic reserve() (which previously aborted the process).
TEST_F(MeshGroupTest, BinaryStlSmallerThanHeaderIsRejected)
{
    Settings& settings = Application::getInstance().current_slice_->scene.settings;
    MeshGroup mesh_group;
    // 50 bytes, not starting with "solid", so it is routed to the binary loader.
    const std::string path = writeTempFile("curaengine_short.stl", std::vector<uint8_t>(50, 0x01));
    EXPECT_FALSE(loadMeshIntoMeshGroup(&mesh_group, path.c_str(), Matrix4x3D(), settings));
    std::filesystem::remove(path);
}

// Regression: a header triangle count that disagrees with the file size must not change what is parsed.
// The loader trusts the size-derived count and reads no out-of-bounds data.
TEST_F(MeshGroupTest, BinaryStlWithInconsistentHeaderCountStillLoads)
{
    Settings& settings = Application::getInstance().current_slice_->scene.settings;
    MeshGroup mesh_group;
    const std::vector<std::array<float, 9>> triangles = {
        { 0, 0, 0, 10, 0, 0, 0, 10, 0 }, // well-separated, non-degenerate
        { 0, 0, 20, 10, 0, 20, 0, 10, 20 },
    };
    const std::string path = writeTempFile("curaengine_badcount.stl", makeBinaryStl(9999, triangles)); // header lies: 9999
    ASSERT_TRUE(loadMeshIntoMeshGroup(&mesh_group, path.c_str(), Matrix4x3D(), settings));
    ASSERT_EQ(mesh_group.meshes.size(), size_t(1));
    EXPECT_EQ(mesh_group.meshes.back().faces_.size(), size_t(2)); // size-derived count, not the bogus header count
    std::filesystem::remove(path);
}

// Behaviour preservation: the memcpy-based decoder still loads the real binary STL fixture.
TEST_F(MeshGroupTest, BinaryStlFixtureLoads)
{
    Settings& settings = Application::getInstance().current_slice_->scene.settings;
    MeshGroup mesh_group;
    const std::string path = std::filesystem::path(__FILE__).parent_path().append("testModel.stl").string();
    ASSERT_TRUE(loadMeshIntoMeshGroup(&mesh_group, path.c_str(), Matrix4x3D(), settings));
    ASSERT_EQ(mesh_group.meshes.size(), size_t(1));
    EXPECT_GT(mesh_group.meshes.back().faces_.size(), size_t(0));
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
