//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SCENE_H
#define SCENE_H

#include "MeshGroup.h" //To store the mesh groups in the scene.
#include "settings/Settings.h" //To store the global settings.

namespace cura
{

/*
 * Represents a scene that should be sliced.
 */
class Scene
{
public:
    /*
     * Create an empty scene.
     *
     * This scene will have no models in it, no extruders, no settings, no
     * nothing.
     */
    Scene();

    /*
     * The global settings in the scene.
     */
    Settings settings;

    /*
     * The mesh groups in the scene.
     */
    std::vector<MeshGroup> mesh_groups;

    /*
     * The extruders in the scene.
     */
    std::vector<ExtruderTrain> extruders;

private:
    /*
     * You are not allowed to copy the scene.
     */
    Scene(const Scene&) = delete;

    /*
     * You are not allowed to copy by assignment either.
     */
    Scene& operator =(const Scene&) = delete;
};

} //namespace cura

#endif //SCENE_H