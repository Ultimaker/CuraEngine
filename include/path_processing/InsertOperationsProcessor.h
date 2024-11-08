// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H
#define PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H

#include <vector>

#include "path_planning/PrintOperationSequence.h"
#include "path_processing/PrintOperationProcessor.h"

namespace cura
{

class PrintOperation;

class InsertOperationsProcessor : public PrintOperationProcessor<PrintOperationSequence>
{
public:
    void process(PrintOperationSequence* operation) final;

protected:
    virtual bool firstOperationMatches(const std::shared_ptr<PrintOperation>& operation) = 0;

    virtual bool secondOperationMatches(const std::shared_ptr<PrintOperation>& first_operation, const std::shared_ptr<PrintOperation>& operation) = 0;

    virtual std::shared_ptr<PrintOperation> makeOperation(const std::shared_ptr<PrintOperation>& operation_first, const std::shared_ptr<PrintOperation>& operation_second) = 0;
};

} // namespace cura

#endif // PATHPROCESSING_INSERTOPERATIONSPROCESSOR_H
