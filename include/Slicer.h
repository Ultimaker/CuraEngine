// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "SlicerLayer.h"
#include "geometry/Point2F.h"
#include "settings/SlicingTolerance.h"

/*
    The Slicer creates layers of polygons from an optimized 3D model.
    The result of the Slicer is a list of polygons without any order or structure.
*/
namespace curaengine
{
class AdaptiveLayer;
class Mesh;
class Point3D;

class Slicer
{
public:
    std::vector<SlicerLayer> layers;

    const Mesh* mesh = nullptr; //!< The sliced mesh

    Slicer(
        Mesh* mesh,
        const coord_t thickness,
        const size_t slice_layer_count,
        bool use_variable_layer_heights,
        std::vector<AdaptiveLayer>* adaptive_layers,
        const SlicingTolerance slicing_tolerance,
        const coord_t initial_layer_thickness);


private:
    /*!
     * \brief Linear interpolation between coordinates of a line.
     *
     * Get the Y of a point with X \p x in the line through (\p x0, \p y0) and
     * (\p x1, \p y1).
     * \param x The X coordinate of the point to find.
     * \param x0 The X coordinate of the first end point of the line segment.
     * \param x1 The X coordinate of the second end point of the line segment.
     * \param y0 The Y coordinate of the first end point of the line segment.
     * \param y1 The Y coordinate of the second end point of the line segment.
     * \return The Y coordinate of the point to find.
     */
    static coord_t interpolate(const coord_t x, const coord_t x0, const coord_t x1, const coord_t y0, const coord_t y1);

    /*!
     * Interpolate the UV coordinates at the 3 points of the given triangle to the given position in barycentric coordinates
     * @param barycentric_coordinates The target position in barycentric coordinates
     * @param uv0 The UV coordinates at point 0 of the triangle
     * @param uv1 The UV coordinates at point 1 of the triangle
     * @param uv2 The UV coordinates at point 2 of the triangle
     * @return The interpolated UV coordinates at the given position
     */
    static Point2F interpolateUV(const Point3D& barycentric_coordinates, const Point2F& uv0, const Point2F& uv1, const Point2F& uv2);

    /*!
     * Get the barycentric coordinates of the given point in the given triangle
     * @param point The 3D point that is inside the given triangle
     * @param p0 The coordinates of point 0 of the triangle
     * @param p1 The coordinates of point 1 of the triangle
     * @param p2 The coordinates of point 2 of the triangle
     * @return The barycentric coordinates of the given point
     */
    static std::optional<Point3D> getBarycentricCoordinates(const Point3LL& point, const Point3LL& p0, const Point3LL& p1, const Point3LL& p2);

    /*!
     * \brief Project a triangle onto a 2D layer.
     *
     * The result is a SlicerSegment object, which is a line segment if the
     * triangle properly intersects the layer, a point if it's an edge case, or
     * nothing if the triangle doesn't intersect the layer.
     * \param p0 A corner of the triangle.
     * \param p1 A corner of the triangle.
     * \param p2 A corner of the triangle.
     * \param z The Z coordinate of the layer to intersect with.
     * \return A slicer segment.
     */
    static SlicerSegment project2D(
        const Point3LL& p0,
        const Point3LL& p1,
        const Point3LL& p2,
        const std::optional<Point2F>& uv0,
        const std::optional<Point2F>& uv1,
        const std::optional<Point2F>& uv2,
        const coord_t z);

    /*! Creates an array of "z bounding boxes" for each face.
     * \param[in] mesh The mesh which is analyzed.
     * \return z heights aka z bounding boxes of the faces.
     */
    static std::vector<std::pair<int32_t, int32_t>> buildZHeightsForFaces(const Mesh& mesh);

    /*! Creates the polygons in layers.
     * \param[in] mesh The mesh which is analyzed.
     * \param[in] slicing_tolerance The way the slicing tolerance should be applied (MIDDLE/INCLUSIVE/EXCLUSIVE).
     * \param[in, out] layers The polygon are created here.
     */
    static void makePolygons(Mesh& mesh, SlicingTolerance slicing_tolerance, std::vector<SlicerLayer>& layers);

    /*! Creates a vector of layers and set their z value.
     * \param[in] mesh The mesh which is analyzed.
     * \param[in] slice_layer_count The amount of layers which shall be sliced.
     * \param[in] slicing_tolerance The way the slicing tolerance should be applied (MIDDLE/INCLUSIVE/EXCLUSIVE).
     * \param[in] initial_layer_thickness Thickness of the first layer.
     * \param[in] thickness Thickness of the layers (apart the first one).
     * \param[in] use_variable_layer_heights Shall we use adaptive layer heights.
     * \param[in] adaptive_layers Adaptive layers (if use_variable_layer_heights).
     * \return layers with set z values.
     */
    static std::vector<SlicerLayer> buildLayersWithHeight(
        size_t slice_layer_count,
        SlicingTolerance slicing_tolerance,
        coord_t initial_layer_thickness,
        coord_t thickness,
        bool use_variable_layer_heights,
        const std::vector<AdaptiveLayer>* adaptive_layers);

    /*! Creates the segments and write them into the layers.
     * \param[in] mesh The mesh which is analyzed.
     * \param[in] zbboxes The z part of the bounding boxes of the faces of the mesh.
     * \param[in] slicing_tolderance Slicing tolerance in order to figure out what happens when vertices are exactly on the slicing boundary.
     * \param[in, out] layers The segments are created here.
     */
    static void
        buildSegments(const Mesh& mesh, const std::vector<std::pair<int32_t, int32_t>>& zbboxes, const SlicingTolerance& slicing_tolerance, std::vector<SlicerLayer>& layers);
};

} // namespace curaengine
