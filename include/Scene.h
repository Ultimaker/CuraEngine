// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "ExtruderTrain.h"
#include "MeshGroup.h"
#include "Settings.h"

namespace curaengine
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
    void processMeshGroup(const std::shared_ptr<MeshGroup>& mesh_group);

    /*!
     * Append a mesh group to this scene and registers its settings parent
     * @param mesh_group The fresh mesh group to be added to the scene
     */
    void appendMeshGroup(const std::shared_ptr<MeshGroup>& mesh_group);

    /*! Remove all the mesh groups from this scene */
    void clearMeshGroups();

    /*! Get all the mesh groups of this scene */
    const std::vector<std::shared_ptr<MeshGroup>>& getMeshGroups() const;

    /*!
     * Get the mesh group at the given index
     * @param index The index of the mesh group, which must be valid
     */
    const std::shared_ptr<MeshGroup>& getMeshGroup(const size_t index) const;

    /*! Check whether the mesh group currently being processed is the first one of the scene */
    bool isCurrentMeshGroupFirst() const;

    /*!
     * Get the mesh group that comes right before the given one
     * @param mesh_group A mesh group of this scene
     * @return The previous mesh group, or nullptr if the given one is the first one or is not part of the scene
     */
    std::shared_ptr<MeshGroup> getPreviousMeshGroup(const std::shared_ptr<MeshGroup>& mesh_group) const;

private:
    /* \brief The mesh groups in the scene. */
    std::vector<std::shared_ptr<MeshGroup>> mesh_groups_;

    /*
     * \brief You are not allowed to copy the scene.
     */
    Scene(const Scene&) = delete;

    /*
     * \brief You are not allowed to copy by assignment either.
     */
    Scene& operator=(const Scene&) = delete;
};

} // namespace curaengine
