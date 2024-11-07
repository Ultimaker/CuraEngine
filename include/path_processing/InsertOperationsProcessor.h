// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H
#define PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H

#include <vector>

#include "path_processing/PrintOperationProcessor.h"

namespace cura
{

class PrintOperation;

template<class OperationType, class ChildOperationType>
class InsertOperationsProcessor : public PrintOperationProcessor<OperationType>
{
public:
    void process(OperationType* operation) override;

protected:
    virtual std::shared_ptr<PrintOperation> makeOperation(const std::shared_ptr<ChildOperationType>& operation_before, const std::shared_ptr<ChildOperationType>& operation_after)
        = 0;
};

template<class OperationType, class ChildOperationType>
void InsertOperationsProcessor<OperationType, ChildOperationType>::process(OperationType* operation)
{
    std::vector<std::shared_ptr<PrintOperation>>& child_operations = operation->getOperations();
    if (child_operations.size() >= 2)
    {
        for (size_t index = 0; index < child_operations.size() - 1; ++index)
        {
            const auto operation_before = std::dynamic_pointer_cast<ChildOperationType>(child_operations[index]);
            const auto operation_after = std::dynamic_pointer_cast<ChildOperationType>(child_operations[index + 1]);

            if (operation_before && operation_after)
            {
                if (const std::shared_ptr<PrintOperation> operation_to_insert = makeOperation(operation_before, operation_after))
                {
                    child_operations.insert(std::next(child_operations.begin(), index + 1), operation_to_insert);
                    index += 1;
                }
            }
        }
    }
}

} // namespace cura

#endif // PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H
