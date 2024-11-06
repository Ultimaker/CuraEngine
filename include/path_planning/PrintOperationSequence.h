// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_PRINTOPERATIONSEQUENCE_H
#define PATHPLANNING_PRINTOPERATIONSEQUENCE_H

#include "path_planning/PrintOperation.h"

namespace cura
{

class ExtruderMove;
class SliceMeshStorage;
class PathExporter;
class LayerPlan;

class PrintOperationSequence : public PrintOperation
{
public:
    bool empty() const noexcept;

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    virtual void write(PathExporter& exporter, const std::vector<const PrintOperation*>& parents = {}) const override;

    std::shared_ptr<PrintOperation> findOperation(const std::function<bool(const std::shared_ptr<PrintOperation>&)>& search_function) const;

    template<class OperationType>
    std::shared_ptr<OperationType> findOperationByType() const;

protected:
    void appendOperation(const std::shared_ptr<PrintOperation>& operation);

private:
    std::vector<std::shared_ptr<PrintOperation>> operations_;
};

template<class OperationType>
std::shared_ptr<OperationType> PrintOperationSequence::findOperationByType() const
{
    std::shared_ptr<PrintOperation> found_operation = findOperation(
        [](const std::shared_ptr<PrintOperation>& operation)
        {
            return static_cast<bool>(std::dynamic_pointer_cast<OperationType>(operation));
        });

    if (found_operation)
    {
        return std::static_pointer_cast<OperationType>(found_operation);
    }

    return nullptr;
}

} // namespace cura

#endif // PATHPLANNING_PRINTOPERATIONSEQUENCE_H
