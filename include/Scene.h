// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SCENE_H
#define SCENE_H

#include "ExtruderTrain.h" //To store the extruders in the scene.
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
     * \brief The global settings in the scene.
     */
    Settings settings;

    /*
     * \brief Which extruder to evaluate each setting on, if different from the
     * normal extruder of the object it's evaluated for.
     */
    std::unordered_map<std::string, ExtruderTrain*> limit_to_extruder;

    /*
     * \brief The mesh groups in the scene.
     */
    std::vector<std::shared_ptr<MeshGroup>> mesh_groups;

    /*
     * \brief The extruders in the scene.
     */
    std::vector<ExtruderTrain> extruders;

    /*
     * \brief The mesh group that is being processed right now.
     *
     * During initialisation this may be nullptr. For the most part, during the
     * slicing process, you can be assured that this will not be null so you can
     * safely dereference it.
     */
    std::shared_ptr<MeshGroup> current_mesh_group;

    /*
     * \brief Create an empty scene.
     *
     * This scene will have no models in it, no extruders, no settings, no
     * nothing.
     */
    Scene();

    /*
     * \brief Gets a string that contains all settings.
     *
     * This string mimics the command line call of CuraEngine. In theory you
     * could call CuraEngine with this output in the command in order to
     * reproduce the output.
     */
    const std::string getAllSettingsString() const;

    /*
     * \brief Generate the 3D printing instructions to print a given mesh group.
     * \param mesh_group The mesh group to slice.
     */
    void processMeshGroup(MeshGroup& mesh_group);

    /*!
     * Append a mesh group to this scene and registers its settings parent
     * @param mesh_group The fresh mesh group to be added to the scene
     */
    void appendMeshGroup(const std::shared_ptr<MeshGroup>& mesh_group);

private:
    /*
     * \brief You are not allowed to copy the scene.
     */
    Scene(const Scene&) = delete;

    /*
     * \brief You are not allowed to copy by assignment either.
     */
    Scene& operator=(const Scene&) = delete;
};

} // namespace cura

#endif // SCENE_H