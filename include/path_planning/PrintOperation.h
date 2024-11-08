// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_PRINTOPERATION_H
#define PATHPLANNING_PRINTOPERATION_H

#include <memory>
#include <vector>

#include <range/v3/view/reverse.hpp>
#include <spdlog/spdlog.h>

namespace cura
{

class PathExporter;
class LayerPlan;
class ExtruderMoveSequence;
class Point3LL;

class PrintOperation
{
public:
    virtual ~PrintOperation() = default; // Force class being polymorphic

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    virtual void write(PathExporter& exporter, const std::vector<const PrintOperation*>& parents) const = 0;

    virtual void applyProcessors(const std::vector<const PrintOperation*>& parents = {});

    virtual std::optional<Point3LL> findStartPosition() const;

    virtual std::optional<Point3LL> findEndPosition() const;

protected:
    template<class ParentType>
    static const ParentType* findParent(const std::vector<const PrintOperation*>& parents, bool warn_not_found = true);
};

template<class ParentType>
const ParentType* PrintOperation::findParent(const std::vector<const PrintOperation*>& parents, bool warn_not_found)
{
    // Parents accumulate top-to-bottom, and we want to find the closest parent from the bottom, so loop reversed
    for (const PrintOperation* parent : parents | ranges::views::reverse)
    {
        auto casted_parent = dynamic_cast<const ParentType*>(parent);
        if (casted_parent)
        {
            return casted_parent;
        }
    }

    if (warn_not_found)
    {
        spdlog::warn("Couldn't find parent of type {}", typeid(ParentType).name());
    }

    return nullptr;
}

} // namespace cura

#endif // PATHPLANNING_PRINTOPERATION_H
