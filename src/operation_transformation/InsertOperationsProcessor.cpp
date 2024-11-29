// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/InsertOperationsProcessor.h"

#include "print_operation/PrintOperation.h"

namespace cura
{

void InsertOperationsProcessor::process(PrintOperationSequence* operation)
{
    std::vector<std::shared_ptr<PrintOperation>>& child_operations = operation->getOperations();
    for (size_t index_first = 0; index_first < child_operations.size(); ++index_first)
    {
        const std::shared_ptr<PrintOperation>& operation_first = child_operations[index_first];
        if (firstOperationMatches(operation_first))
        {
            for (size_t index_second = index_first + 1; index_second < child_operations.size(); ++index_second)
            {
                const std::shared_ptr<PrintOperation>& operation_second = child_operations[index_second];
                if (secondOperationMatches(operation_first, operation_second))
                {
                    if (const std::shared_ptr<PrintOperation> operation_to_insert = makeOperation(operation_first, operation_second))
                    {
                        child_operations.insert(std::next(child_operations.begin(), index_second), operation_to_insert);
                        index_first = index_second;
                    }
                    else
                    {
                        index_first = index_second - 1;
                    }
                    break;
                }
            }
        }
    }
}

} // namespace cura
