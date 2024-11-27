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
class ContinuousExtruderMoveSequence;
class Point3LL;
class PrintOperationSequence;

class PrintOperation
{
public:
    virtual ~PrintOperation() = default; // Force class being polymorphic

    std::shared_ptr<PrintOperationSequence> getParent() const;

    void setParent(std::weak_ptr<PrintOperationSequence> parent);

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    virtual void write(PathExporter& exporter) const = 0;

    virtual void applyProcessors(const std::vector<const PrintOperation*>& parents = {});

    virtual std::optional<Point3LL> findStartPosition() const;

    virtual std::optional<Point3LL> findEndPosition() const;

protected:
    std::shared_ptr<PrintOperationSequence>
        findParent(const std::function<bool(const std::shared_ptr<PrintOperationSequence>&)>& search_function, bool warn_not_found = true) const;

    template<class ParentType>
    std::shared_ptr<ParentType> findParentByType(bool warn_not_found = true) const;

private:
    std::weak_ptr<PrintOperationSequence> parent_;
};

template<class ParentType>
std::shared_ptr<ParentType> PrintOperation::findParentByType(bool warn_not_found) const
{
    return std::reinterpret_pointer_cast<ParentType>(findParent(
        [](const std::shared_ptr<PrintOperationSequence>& parent)
        {
            return static_cast<bool>(std::dynamic_pointer_cast<ParentType>(parent));
        },
        warn_not_found));
}

} // namespace cura

#endif // PATHPLANNING_PRINTOPERATION_H
