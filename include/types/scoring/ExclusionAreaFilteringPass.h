// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>
#include <vector>

#include "types/geometry/Shape.h"
#include "types/scoring/FilteringPass.h"

namespace curaengine::types::geometry
{
class Shape;
} // namespace curaengine::types::geometry

namespace curaengine::types::scoring
{

/*!
 * Pass that removes the candidates that are located inside an exclusion area, as long as there is at least one candidate that is outside of it. If all the
 * candidates are inside, they are all kept.
 * The code is in the cpp file, so this class can only be used with the types of elements that are instantiated at the end of it.
 * \tparam T The type of the elements, which must have a position (see getElementPosition())
 */
template<typename T>
class ExclusionAreaFilteringPass : public FilteringPass<T>
{
private:
    const types::geometry::Shape exclusion_area_;

public:
    /*!
     * \param exclusion_area The area to be excluded, which is shared with the criterion so that it lives as long as the criterion does. To use an area that is owned by someone
     * else, a non-owning pointer can be given (e.g. with the aliasing constructor of std::shared_ptr)
     */
    explicit ExclusionAreaFilteringPass(const types::geometry::Shape& exclusion_area);

    explicit ExclusionAreaFilteringPass(types::geometry::Shape&& exclusion_area);

protected:
    void filter(std::vector<Candidate<T>>& candidates) const override;
};

} // namespace curaengine::types::scoring
