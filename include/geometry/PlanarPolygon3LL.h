// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <execution>

#include <boost/unordered/concurrent_flat_map.hpp>

#include "utils/Coord_t.h"


namespace curaengine::geometry
{

class Segment3LL;

/*!
 * Represents a polygon that is contained in a plan, but has 3D coordinates. So the plan can be non-axis-aligned.
 */
class PlanarPolygon3LL
{
public:
    explicit PlanarPolygon3LL(std::vector<Segment3LL>&& segments);

    explicit PlanarPolygon3LL(const std::initializer_list<Segment3LL>& segments);

    /*!
     * Intersects the polygon with a "layer" on the X plane, e.g. a pair or YZ plans
     * @param layer_start_x The X coordinate of the lowest YZ plane
     * @param layer_end_x The X coordinate of the highest YZ plane
     * @return The polygon that is a sub-part of the current polygon with the layer, or nullopt if the intersection is empty
     */
    std::optional<PlanarPolygon3LL> intersectionWithXLayer(const ::curaengine::utils::coord_t layer_start_x, const ::curaengine::utils::coord_t layer_end_x) const;

    /*!
     * Intersects the polygon with a "layer" on the Y plane, e.g. a pair or XZ plans
     * @param layer_start_y The Y coordinate of the lowest XZ plane     * @param layer_end_y The Y coordinate of the highest XZ plane
     * @return The polygon that is a sub-part of the current polygon with the layer, or nullopt if the intersection is empty
     */
    std::optional<PlanarPolygon3LL> intersectionWithYLayer(const ::curaengine::utils::coord_t layer_start_y, const ::curaengine::utils::coord_t layer_end_y) const;

    /*!
     * Intersects the polygon with a "layer" on the Z plane, e.g. a pair or XY plans
     * @param layer_start_z The Z coordinate of the lowest XY plane
     * @param layer_end_z The Z coordinate of the highest XY plane
     * @return The polygon that is a sub-part of the current polygon with the layer, or nullopt if the intersection is empty
     */
    std::optional<PlanarPolygon3LL> intersectionWithZLayer(const ::curaengine::utils::coord_t layer_start_z, const ::curaengine::utils::coord_t layer_end_z) const;

    std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> minmaxX() const;

    std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> minmaxY() const;

    std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> minmaxZ() const;

private:
    std::tuple<::curaengine::utils::coord_t, ::curaengine::utils::coord_t> minmax(const std::function<::curaengine::utils::coord_t(const Segment3LL& segment)>& get_coordinate) const;

    /*!
     * Intersects the polygon with a axis-aligned "layer"
     * @param layer_start The coordinate of the lowest plane
     * @param layer_end The coordinate of the highest plane
     * @param function_intersect_with_layer The segment method to be called to intersect with the layer on the proper axis
     * @return The polygon that is a sub-part of the current polygon with the layer, or nullopt if the intersection is empty
     */
    std::optional<PlanarPolygon3LL> intersectionWithLayer(
        const ::curaengine::utils::coord_t layer_start,
        const ::curaengine::utils::coord_t layer_end,
        const std::function<std::optional<Segment3LL>(const Segment3LL&, const ::curaengine::utils::coord_t, const ::curaengine::utils::coord_t)>& function_intersect_with_layer) const;

private:
    std::vector<Segment3LL> segments_;
};

} // namespace curaengine::geometry
