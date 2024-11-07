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

void PrintOperationSequence::applyProcessors()
{
    PrintOperation::applyProcessors();

    for (const std::shared_ptr<PrintOperation>& operation : operations_)
    {
        operation->applyProcessors();
    }
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

    switch (search_order)
    {
    case SearchOrder::Forward:
        return find_in(operations_.begin(), operations_.end());
    case SearchOrder::Backward:
        return find_in(operations_.rbegin(), operations_.rend());
    case SearchOrder::DepthFirst:
        for (const std::shared_ptr<PrintOperation>& operation : operations_)
        {
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
    }

    return nullptr;
}

const std::vector<std::shared_ptr<PrintOperation>>& PrintOperationSequence::getOperations() const noexcept
{
    return operations_;
}

std::vector<std::shared_ptr<PrintOperation>>& PrintOperationSequence::getOperations() noexcept
{
    return operations_;
}

} // namespace cura
