// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/PrintOperationSequence.h"

#include <range/v3/algorithm/contains.hpp>
#include <range/v3/algorithm/copy.hpp>
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

void PrintOperationSequence::applyProcessors()
{
    PrintOperation::applyProcessors();

    for (const PrintOperationPtr& operation : operations_)
    {
        operation->applyProcessors();
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
    if (! operation)
    {
        return;
    }

    if (std::shared_ptr<PrintOperationSequence> actual_parent = operation->getParent())
    {
        if (actual_parent.get() == this)
        {
            return;
        }

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
#warning This probably doesn't work
        ranges::views::remove(operations_, operation);
    }
    else
    {
        spdlog::error("Trying to remove an operation that is currently not a child");
    }
}

void PrintOperationSequence::insertOperationAfter(const PrintOperationPtr& actual_operation, const PrintOperationPtr& insert_operation)
{
    if (actual_operation->getParent().get() == this)
    {
        const std::shared_ptr<PrintOperationSequence> actual_parent = insert_operation->getParent();
        if (actual_parent.get() == this)
        {
            std::vector<PrintOperationPtr>::iterator insert_actual_position = ranges::find(operations_, insert_operation);
            if (insert_actual_position != operations_.end())
            {
                operations_.erase(insert_actual_position);
            }
            else
            {
                spdlog::error("Operation to insert is registered has child but could not be found in children list");
            }
        }
        else
        {
            if (actual_parent)
            {
                actual_parent->removeOperation(insert_operation);
            }
            insert_operation->setParent(weak_from_this());
        }

        auto actual_iterator = ranges::find(operations_, actual_operation);
        if (actual_iterator != operations_.end())
        {
            operations_.insert(std::next(actual_iterator), insert_operation);
        }
        else
        {
            spdlog::error("Actual operation is registered has child but could not be found in children list");
        }
    }
    else
    {
        spdlog::error("The given actual_operation is not a registered operation");
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

std::vector<PrintOperationPtr> PrintOperationSequence::findOperations(
    const std::function<bool(const PrintOperationPtr&)>& search_function,
    const SearchOrder search_order,
    const std::optional<size_t> max_depth) const
{
    std::vector<PrintOperationPtr> found_operations;

    if (! max_depth.has_value() || max_depth.value() > 0)
    {
        const std::optional<size_t> next_depth = max_depth.has_value() ? max_depth.value() - 1 : max_depth;

        const auto find_depth_first = [&search_function, &search_order, &next_depth, &found_operations](auto begin, auto end) -> void
        {
            for (auto iterator = begin; iterator != end; ++iterator)
            {
                const PrintOperationPtr& operation = *iterator;
                if (search_function(operation))
                {
                    found_operations.push_back(operation);
                }

                if (const auto operation_sequence = std::dynamic_pointer_cast<PrintOperationSequence>(operation))
                {
                    ranges::copy(operation_sequence->findOperations(search_function, search_order, next_depth), std::back_inserter(found_operations));
                }
            }
        };

        switch (search_order)
        {
        case SearchOrder::Forward:
            find_depth_first(operations_.begin(), operations_.end());
            break;
        case SearchOrder::Backward:
            find_depth_first(operations_.rbegin(), operations_.rend());
            break;
        }
    }
    else
    {
        auto find_in = [&search_function, &found_operations](auto begin, auto end) -> void
        {
            ranges::copy_if(begin, end, std::back_inserter(found_operations), search_function);
        };

        switch (search_order)
        {
        case SearchOrder::Forward:
            find_in(operations_.begin(), operations_.end());
            break;
        case SearchOrder::Backward:
            find_in(operations_.rbegin(), operations_.rend());
            break;
        }
    }

    return found_operations;
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
