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

void PrintOperationSequence::appendOperation(const std::shared_ptr<PrintOperation>& operation)
{
    operations_.push_back(operation);
}

} // namespace cura
