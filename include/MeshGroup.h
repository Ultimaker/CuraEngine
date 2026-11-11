// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <filesystem>
#include <optional>

#include "Mesh.h"

namespace curaengine::types::geometry
{
class Matrix4x3D;
}

namespace curaengine::types
{
struct Ratio;
}

namespace curaengine
{


/*!
 * A MeshGroup is a collection with 1 or more 3D meshes.
 *
 * One MeshGroup is a whole which is printed at once.
 * Generally there is one single MeshGroup, though when using one-at-a-time printing, multiple MeshGroups are processed consecutively.
 */
class MeshGroup
{
public:
    MeshGroup() = default;

    ~MeshGroup() = default;

    MeshGroup(MeshGroup&& other) noexcept = default;

    MeshGroup& operator=(MeshGroup&& other) noexcept = default;

    /* Copying a MeshGroup is not allowed */
    MeshGroup(const MeshGroup& other) = delete;
    MeshGroup& operator=(const MeshGroup& other) = delete;

    std::vector<Mesh> meshes;
    Settings settings;
    bool has_painted_support = false;

    std::optional<int> num_extruders_used = std::nullopt;

    types::geometry::Point3LL min() const; //! minimal corner of bounding box
    types::geometry::Point3LL max() const; //! maximal corner of bounding box

    void clear();

    void finalize();

    /*!
     * Scale the entire mesh group, with the bottom center as origin point.
     *
     * The mesh group is scaled around the bottom center of its bounding box. So
     * that's the center in the X and Y directions, but Z=0. This simulates the
     * shrinkage while sticking to the build plate.
     */
    void scaleFromBottom(const types::Ratio factor_xy, const types::Ratio factor_z);

    /*!
     * Wether or not the mesh-skipping feature is enabled.
     *
     * Currently, we don't support > 1 extruders when skipping meshes.
     * Also, it's a bit of a waste to add the gcode-comments when there's only one mesh, so we don't.
     */
    bool getSkipMeshEnabled() const;
};

/*!
 * Load a Mesh from file and store it in the \p meshgroup.
 *
 * \param meshgroup The meshgroup where to store the mesh
 * \param filename The filename of the mesh file
 * \param transformation The transformation applied to all vertices
 * \param object_parent_settings (optional) The parent settings object of the new mesh. Defaults to \p meshgroup if none is given.
 * \return whether the file could be loaded
 */
bool loadMeshIntoMeshGroup(MeshGroup* meshgroup, const std::filesystem::path& filename, const types::geometry::Matrix4x3D& transformation, Settings& object_parent_settings);

bool loadMeshOBJ(Mesh* mesh, const std::string& filename, const types::geometry::Matrix4x3D& matrix);

} // namespace curaengine
