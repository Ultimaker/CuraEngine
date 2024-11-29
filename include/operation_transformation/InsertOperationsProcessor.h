// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H
#define PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H

#include <vector>

#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/PrintOperationSequence.h"

namespace cura
{

class PrintOperation;

#warning obsolete for now, but contains some code that may be reused at some point
class InsertOperationsProcessor : public PrintOperationTransformer<PrintOperationSequence>
{
public:
    void process(PrintOperationSequence* operation) override;

protected:
    virtual bool firstOperationMatches(const std::shared_ptr<PrintOperation>& operation) = 0;

    virtual bool secondOperationMatches(const std::shared_ptr<PrintOperation>& first_operation, const std::shared_ptr<PrintOperation>& operation) = 0;

    virtual std::shared_ptr<PrintOperation> makeOperation(const std::shared_ptr<PrintOperation>& operation_first, const std::shared_ptr<PrintOperation>& operation_second) = 0;
};

} // namespace cura

#endif // PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H
