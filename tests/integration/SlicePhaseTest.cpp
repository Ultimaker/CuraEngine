//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/Application.h" //To set up a slice with settings.
#include "../src/Slice.h" //To set up a scene to slice.
#include "../src/utils/floatpoint.h" //For FMatrix3x3 to load STL files.

namespace cura
{

/*
 * Integration test on the slicing phase of CuraEngine. This tests if the
 * slicing algorithm correctly splits a 3D model up into 2D layers.
 */
class SlicePhaseTest : public testing::Test
{
    void SetUp()
    {
        //Set up a scene so that we may request settings.
        Application::getInstance().current_slice = new Slice(1);
    }
};

TEST_F(SlicePhaseTest, Cube)
{
    Scene& scene = Application::getInstance().current_slice->scene;
    const FMatrix3x3 transformation;
    //Path to cube.stl is relative to CMAKE_CURRENT_SOURCE_DIR/tests
    ASSERT_TRUE(loadMeshIntoMeshGroup(&scene.mesh_groups.back(), "integration/resources/cube.stl", transformation, scene.settings));
}

} //namespace cura