// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/PrintOperationSequence.h"

#include <range/v3/algorithm/contains.hpp>
#include <range/v3/algorithm/copy_if.hpp>
#include <range/v3/algorithm/remove_if.hpp>
#include <range/v3/view/remove.hpp>
#include <range/v3/view/set_algorithm.hpp>

#include "print_operation/PrintOperationPtr.h"

namespace cura
{

bool PrintOperationSequence::empty() const noexcept
{
    return operations_.empty();
}

void PrintOperationSequence::write(PlanExporter& exporter) const
{
    for (const PrintOperationPtr& operation : operations_)
    {
        operation->write(exporter);
    }
}

void PrintOperationSequence::applyProcessors(const std::vector<const PrintOperation*>& parents)
{
    PrintOperation::applyProcessors(parents);

    std::vector<const PrintOperation*> new_parents = parents;
    new_parents.push_back(this);

    for (const PrintOperationPtr& operation : operations_)
    {
        operation->applyProcessors(new_parents);
    }
}

std::optional<Point3LL> PrintOperationSequence::findStartPosition() const
{
    for (const PrintOperationPtr& operation : operations_)
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
    for (const PrintOperationPtr& operation : operations_ | ranges::views::reverse)
    {
        std::optional<Point3LL> end_position = operation->findEndPosition();
        if (end_position.has_value())
        {
            return end_position;
        }
    }

    return std::nullopt;
}

void PrintOperationSequence::appendOperation(const PrintOperationPtr& operation)
{
    if (std::shared_ptr<PrintOperationSequence> actual_parent = operation->getParent())
    {
        actual_parent->removeOperation(operation);
    }

    operations_.push_back(operation);
    operation->setParent(weak_from_this());
}

void PrintOperationSequence::removeOperation(const PrintOperationPtr& operation)
{
    if (operation->getParent() == shared_from_this())
    {
        operation->setParent({});
        ranges::views::remove(operations_, operation);
    }
    else
    {
        spdlog::error("Trying to remove an operation that is currently not a child");
    }
}

PrintOperationPtr PrintOperationSequence::findOperation(
    const std::function<bool(const PrintOperationPtr&)>& search_function,
    const SearchOrder search_order,
    const std::optional<size_t> max_depth) const
{
    if (! max_depth.has_value() || max_depth.value() > 0)
    {
        const std::optional<size_t> next_depth = max_depth.has_value() ? max_depth.value() - 1 : max_depth;

        const auto find_depth_first = [&search_function, &search_order, &next_depth](auto begin, auto end) -> PrintOperationPtr
        {
            for (auto iterator = begin; iterator != end; ++iterator)
            {
                const PrintOperationPtr& operation = *iterator;
                if (search_function(operation))
                {
                    return operation;
                }

                if (const auto operation_sequence = std::dynamic_pointer_cast<PrintOperationSequence>(operation))
                {
                    if (auto result = operation_sequence->findOperation(search_function, search_order, next_depth))
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
            return find_depth_first(operations_.begin(), operations_.end());
        case SearchOrder::Backward:
            return find_depth_first(operations_.rbegin(), operations_.rend());
        }
    }
    else
    {
        auto find_in = [&search_function](auto begin, auto end) -> PrintOperationPtr
        {
            auto iterator = std::find_if(begin, end, search_function);
            if (iterator != end)
            {
                return *iterator;
            }

            return nullptr;
        };

        switch (search_order)
        {
        case SearchOrder::Forward:
            return find_in(operations_.begin(), operations_.end());
        case SearchOrder::Backward:
            return find_in(operations_.rbegin(), operations_.rend());
        }
    }

    return nullptr;
}

const std::vector<PrintOperationPtr>& PrintOperationSequence::getOperations() const noexcept
{
    return operations_;
}

// void PrintOperationSequence::setOperations(std::vector<PrintOperationPtr>& operations) noexcept
// {
//     for (const PrintOperationPtr& removed_operation : operations_)
//     {
//         if (! ranges::contains(operations, removed_operation))
//         {
//             removed_operation->setParent({});
//         }
//     }
//
//     for (const PrintOperationPtr& added_operation : operations)
//     {
//         if (! ranges::contains(operations_, added_operation))
//         {
//             added_operation->setParent(weak_from_this());
//         }
//     }
//
//     operations_ = operations;
// }

} // namespace cura
