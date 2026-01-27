// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MESH_UTILS_H
#define UTILS_MESH_UTILS_H

#include <optional>
#include <string>
#include <vector>

#include "geometry/Triangle2F.h"
#include "geometry/Triangle3D.h"

namespace cura
{

class Point3D;
class Mesh;

namespace MeshUtils
{

std::optional<Point3D> getBarycentricCoordinates(const Point3D& point, const Triangle3D& triangle);

Point2F getUVCoordinates(const Point3D& barycentric_coordinates, const Triangle2F& uv_coordinates);

/*!
 * Load texture data from PNG binary data and attach it to a mesh.
 * This function handles PNG parsing, metadata extraction, and texture attachment.
 *
 * @param texture_data Binary PNG data as bytes
 * @param mesh The mesh to attach the texture to
 * @param source_description Description of the texture source for logging (e.g., filename or "network data")
 * @param log_no_metadata Whether to log when no metadata is found (file loading) vs silently return (network data)
 * @return true if the texture was loaded successfully with valid metadata, false otherwise
 */
bool loadTextureFromPngData(const std::vector<unsigned char>& texture_data, Mesh& mesh, const std::string& source_description, bool log_no_metadata = true);

/*!
 * Load texture data from a PNG file and attach it to a mesh.
 * Convenience wrapper around loadTextureFromPngData for file-based loading.
 *
 * @param mesh The mesh to attach the texture to
 * @param texture_filename The path to the PNG texture file
 * @return true if the texture was loaded successfully, false otherwise
 */
bool loadTextureFromFile(Mesh& mesh, const std::string& texture_filename);

/*!
 * Load texture data from PNG data provided as a string and attach it to a mesh.
 * Convenience wrapper around loadTextureFromPngData for string-based loading.
 *
 * @param texture_str Binary PNG data as string
 * @param mesh The mesh to attach the texture to
 */
void loadTextureFromString(const std::string& texture_str, Mesh& mesh);

} // namespace MeshUtils

} // namespace cura

#endif
