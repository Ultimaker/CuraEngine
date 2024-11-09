// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_PRINTOPERATIONSEQUENCE_H
#define PATHPLANNING_PRINTOPERATIONSEQUENCE_H

#include "geometry/Point3LL.h"
#include "path_planning/PrintOperation.h"
#include "path_processing/PrintOperationProcessor.h"

namespace cura
{

class ExtruderMove;
class SliceMeshStorage;
class PathExporter;
class LayerPlan;

class PrintOperationSequence : public PrintOperation
{
public:
    enum class SearchOrder
    {
        Forward, // Only search in direct children, forwards
        Backward, // Only search in direct children, backwards
        DepthFirstForward, // Search in children tree, depth-first forwards
        DepthFirstBackward, // Search in children tree, depth-first backwards
    };

public:
    bool empty() const noexcept;

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    void write(PathExporter& exporter, const std::vector<const PrintOperation*>& parents = {}) const override;

    void applyProcessors(const std::vector<const PrintOperation*>& parents = {}) override;

    std::optional<Point3LL> findStartPosition() const override;

    std::optional<Point3LL> findEndPosition() const override;

    std::shared_ptr<PrintOperation>
        findOperation(const std::function<bool(const std::shared_ptr<PrintOperation>&)>& search_function, const SearchOrder search_order = SearchOrder::Forward) const;

    template<class OperationType>
    std::shared_ptr<OperationType> findOperationByType(const SearchOrder search_order = SearchOrder::Forward) const;

    std::shared_ptr<PrintOperation>
        findOperation(const std::vector<const PrintOperation*>& parents, const std::function<bool(const std::shared_ptr<PrintOperation>&)>& search_function) const;

    const std::vector<std::shared_ptr<PrintOperation>>& getOperations() const noexcept;

    std::vector<std::shared_ptr<PrintOperation>>& getOperations() noexcept;

    template<class OperationType>
    std::vector<std::shared_ptr<OperationType>> getOperationsAs() noexcept;

    void setOperations(std::vector<std::shared_ptr<PrintOperation>>& operations) noexcept;

protected:
    void appendOperation(const std::shared_ptr<PrintOperation>& operation);

    template<class ChildType>
    void applyProcessorToOperationsRecursively(PrintOperationProcessor<ChildType>& processor);

private:
    std::vector<std::shared_ptr<PrintOperation>> operations_;
};

template<class OperationType>
std::shared_ptr<OperationType> PrintOperationSequence::findOperationByType(const SearchOrder search_order) const
{
    std::shared_ptr<PrintOperation> found_operation = findOperation(
        [](const std::shared_ptr<PrintOperation>& operation)
        {
            return static_cast<bool>(std::dynamic_pointer_cast<OperationType>(operation));
        },
        search_order);

    if (found_operation)
    {
        return std::static_pointer_cast<OperationType>(found_operation);
    }

    return nullptr;
}

template<class OperationType>
std::vector<std::shared_ptr<OperationType>> PrintOperationSequence::getOperationsAs() noexcept
{
    std::vector<std::shared_ptr<OperationType>> result;
    result.reserve(operations_.size());

    for (const std::shared_ptr<PrintOperation>& operation : operations_)
    {
        if (auto operation_as = std::dynamic_pointer_cast<OperationType>(operation)) [[likely]]
        {
            result.push_back(operation_as);
        }
        else
        {
            spdlog::error("Found an child operation which is not of expected type {}, it will be discarded", typeid(operation).name());
        }
    }

    return result;
}

template<class ChildType>
void PrintOperationSequence::applyProcessorToOperationsRecursively(PrintOperationProcessor<ChildType>& processor)
{
    for (const auto& operation : operations_)
    {
        if (auto operation_sequence = std::dynamic_pointer_cast<PrintOperationSequence>(operation))
        {
            operation_sequence->applyProcessorToOperationsRecursively(processor);
        }

        if (auto casted_child = std::dynamic_pointer_cast<ChildType>(operation))
        {
            processor.process(casted_child.get());
        }
    }
}

} // namespace cura

#endif // PATHPLANNING_PRINTOPERATIONSEQUENCE_H
