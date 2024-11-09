// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/PrintOperationSequence.h"

namespace cura
{

bool PrintOperationSequence::empty() const noexcept
{
    return operations_.empty();
}

void PrintOperationSequence::write(PathExporter& exporter, const std::vector<const PrintOperation*>& parents) const
{
    std::vector<const PrintOperation*> new_parents = parents;
    new_parents.push_back(this);

    for (const std::shared_ptr<PrintOperation>& operation : operations_)
    {
        operation->write(exporter, new_parents);
    }
}

void PrintOperationSequence::applyProcessors(const std::vector<const PrintOperation*>& parents)
{
    PrintOperation::applyProcessors(parents);

    std::vector<const PrintOperation*> new_parents = parents;
    new_parents.push_back(this);

    for (const std::shared_ptr<PrintOperation>& operation : operations_)
    {
        operation->applyProcessors(new_parents);
    }
}

std::optional<Point3LL> PrintOperationSequence::findStartPosition() const
{
    for (const std::shared_ptr<PrintOperation>& operation : operations_)
    {
        std::optional<Point3LL> start_position = operation->findStartPosition();
        if (start_position.has_value())
        {
            return start_position;
        }
    }

    return std::nullopt;
}

std::optional<Point3LL> PrintOperationSequence::findEndPosition() const
{
    for (const std::shared_ptr<PrintOperation>& operation : operations_ | ranges::views::reverse)
    {
        std::optional<Point3LL> end_position = operation->findEndPosition();
        if (end_position.has_value())
        {
            return end_position;
        }
    }

    return std::nullopt;
}

void PrintOperationSequence::appendOperation(const std::shared_ptr<PrintOperation>& operation)
{
    operations_.push_back(operation);
}

std::shared_ptr<PrintOperation>
    PrintOperationSequence::findOperation(const std::function<bool(const std::shared_ptr<PrintOperation>&)>& search_function, const SearchOrder search_order) const
{
#warning We should iterator over all children in the tree, depth first
    auto find_in = [&search_function](auto begin, auto end) -> std::shared_ptr<PrintOperation>
    {
        auto iterator = std::find_if(begin, end, search_function);
        if (iterator != end)
        {
            return *iterator;
        }

        return nullptr;
    };

    auto find_depth_first = [&search_function, &search_order](auto begin, auto end) -> std::shared_ptr<PrintOperation>
    {
        for (auto iterator = begin; iterator != end; ++iterator)
        {
            const std::shared_ptr<PrintOperation>& operation = *iterator;
            if (search_function(operation))
            {
                return operation;
            }

            if (const auto operation_sequence = std::dynamic_pointer_cast<PrintOperationSequence>(operation))
            {
                if (auto result = operation_sequence->findOperation(search_function, search_order))
                {
                    return result;
                }
            }
        }

        return nullptr;
    };

    switch (search_order)
    {
    case SearchOrder::Forward:
        return find_in(operations_.begin(), operations_.end());
    case SearchOrder::Backward:
        return find_in(operations_.rbegin(), operations_.rend());
    case SearchOrder::DepthFirstForward:
        return find_depth_first(operations_.begin(), operations_.end());
    case SearchOrder::DepthFirstBackward:
        return find_depth_first(operations_.rbegin(), operations_.rend());
    }

    return nullptr;
}

std::shared_ptr<PrintOperation> PrintOperationSequence::findOperation(
    const std::vector<const PrintOperation*>& parents,
    const std::function<bool(const std::shared_ptr<PrintOperation>&)>& search_function) const
{
    if (! parents.empty())
    {
        const PrintOperation* child = this;

        for (const PrintOperation* parent : parents | ranges::views::reverse)
        {
            if (const auto* parent_as_sequence = dynamic_cast<const PrintOperationSequence*>(parent))
            {
                const std::vector<std::shared_ptr<PrintOperation>>& children = parent_as_sequence->getOperations();
                auto iterator_child = std::find_if(
                    children.begin(),
                    children.end(),
                    [&child](const std::shared_ptr<PrintOperation>& operation)
                    {
                        return operation.get() == child;
                    });
                if (iterator_child != children.begin())
                {
                    for (auto iterator = iterator_child - 1; iterator != children.begin(); --iterator)
                    {
                        if (search_function(*iterator))
                        {
                            return *iterator;
                        }

                        if (const auto child_as_sequence = std::dynamic_pointer_cast<const PrintOperationSequence>(*iterator))
                        {
                            child_as_sequence->findOperation(search_function, SearchOrder::DepthFirstBackward);
                        }
                    }
                }

                child = parent;
            }
        }

        return nullptr;
    }
}

const std::vector<std::shared_ptr<PrintOperation>>& PrintOperationSequence::getOperations() const noexcept
{
    return operations_;
}

std::vector<std::shared_ptr<PrintOperation>>& PrintOperationSequence::getOperations() noexcept
{
    return operations_;
}

void PrintOperationSequence::setOperations(std::vector<std::shared_ptr<PrintOperation>>& operations) noexcept
{
    operations_ = operations;
}

} // namespace cura
