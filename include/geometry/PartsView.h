// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_PARTS_VIEW_H
#define GEOMETRY_PARTS_VIEW_H

#include <vector>

namespace cura
{

class Shape;
class SingleShape;

/*!
 * Extension of vector<vector<unsigned int>> which is similar to a vector of PolygonParts, except the base of the container is indices to polygons into the original Polygons,
 * instead of the polygons themselves
 */
class PartsView : public std::vector<std::vector<size_t>>
{
public:
    Shape& polygons_;

    PartsView() = delete;

    explicit PartsView(Shape& polygons)
        : polygons_{ polygons }
    {
    }

    PartsView(PartsView&& parts_view) = default;
    PartsView(const PartsView& parts_view) = default;

    ~PartsView() = default;

    PartsView& operator=(PartsView&& parts_view) = delete;
    PartsView& operator=(const PartsView& parts_view) = delete;

    /*!
     * Get the index of the SingleShape of which the polygon with index \p poly_idx is part.
     *
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The SingleShape containing the polygon with index \p poly_idx
     */
    size_t getPartContaining(size_t poly_idx, size_t* boundary_poly_idx = nullptr) const;

    /*!
     * Assemble the SingleShape of which the polygon with index \p poly_idx is part.
     *
     * \param poly_idx The index of the polygon in \p polygons
     * \param boundary_poly_idx Optional output parameter: The index of the boundary polygon of the part in \p polygons
     * \return The SingleShape containing the polygon with index \p poly_idx
     */
    SingleShape assemblePartContaining(size_t poly_idx, size_t* boundary_poly_idx = nullptr) const;

    /*!
     * Assemble the SingleShape of which the polygon with index \p poly_idx is part.
     *
     * \param part_idx The index of the part
     * \return The SingleShape with index \p poly_idx
     */
    [[nodiscard]] SingleShape assemblePart(size_t part_idx) const;
};

} // namespace cura

#endif // GEOMETRY_PARTS_VIEW_H
