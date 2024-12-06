// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_PRINTOPERATIONSEQUENCE_H
#define PATHPLANNING_PRINTOPERATIONSEQUENCE_H

#include "geometry/Point3LL.h"
#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/PrintOperation.h"

namespace cura
{

class ExtruderMove;
class SliceMeshStorage;
class PlanExporter;
class LayerPlan;

namespace SearchDepth
{
static constexpr std::optional<size_t> Full = std::nullopt;
static constexpr std::optional<size_t> DirectChildren = 0;
}; // namespace SearchDepth

class PrintOperationSequence : public PrintOperation, public std::enable_shared_from_this<PrintOperationSequence>
{
public:
    enum class SearchOrder
    {
        Forward,
        Backward,
    };

public:
    bool empty() const noexcept;

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    void write(PlanExporter& exporter) const override;

    void applyProcessors(const std::vector<const PrintOperation*>& parents = {}) override;

    std::optional<Point3LL> findStartPosition() const override;

    std::optional<Point3LL> findEndPosition() const override;

    /*!
     * Searches a child operation, recursively or not, forwards of backwards, given a search function
     *
     * @param search_function The search function that should find the matching operation
     * @param search_order Whether we should search forwards or backwards
     * @param max_depth The maximum depth of children to look for. 0 means only direct children, nullopt means full tree. You case use SearchDepth defines.
     * @return The first found operation, or a null ptr if none was found
     * @note This function can also be used to iterate over children by providing a search function that always returns false
     */
    std::shared_ptr<PrintOperation> findOperation(
        const std::function<bool(const std::shared_ptr<PrintOperation>&)>& search_function,
        const SearchOrder search_order = SearchOrder::Forward,
        const std::optional<size_t> max_depth = SearchDepth::DirectChildren) const;

    template<class OperationType>
    std::shared_ptr<OperationType>
        findOperationByType(const SearchOrder search_order = SearchOrder::Forward, const std::optional<size_t> max_depth = SearchDepth::DirectChildren) const;

    const std::vector<std::shared_ptr<PrintOperation>>& getOperations() const noexcept;

    std::vector<std::shared_ptr<PrintOperation>>& getOperations() noexcept;

    template<class OperationType>
    std::vector<std::shared_ptr<OperationType>> getOperationsAs() noexcept;

    void setOperations(std::vector<std::shared_ptr<PrintOperation>>& operations) noexcept;

protected:
    void appendOperation(const std::shared_ptr<PrintOperation>& operation);

    void removeOperation(const std::shared_ptr<PrintOperation>& operation);

    template<class ChildType>
    void applyProcessorToOperationsRecursively(PrintOperationTransformer<ChildType>& processor);

private:
    std::vector<std::shared_ptr<PrintOperation>> operations_;
};

template<class OperationType>
std::shared_ptr<OperationType> PrintOperationSequence::findOperationByType(const SearchOrder search_order, const std::optional<size_t> max_depth) const
{
    std::shared_ptr<PrintOperation> found_operation = findOperation(
        [](const std::shared_ptr<PrintOperation>& operation)
        {
            return static_cast<bool>(std::dynamic_pointer_cast<OperationType>(operation));
        },
        search_order,
        max_depth);

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
void PrintOperationSequence::applyProcessorToOperationsRecursively(PrintOperationTransformer<ChildType>& processor)
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
