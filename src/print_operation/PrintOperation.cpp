// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/PrintOperation.h"

#include "geometry/Point3LL.h"
#include "print_operation/PrintOperationSequence.h"

namespace cura
{

std::shared_ptr<PrintOperationSequence> PrintOperation::getParent() const
{
    return parent_.lock();
}

void PrintOperation::setParent(std::weak_ptr<PrintOperationSequence> parent)
{
    parent_ = parent;
}

void PrintOperation::applyProcessors()
{
    // Default behavior is do nothing
}

std::optional<Point3LL> PrintOperation::findStartPosition() const
{
    return std::nullopt; // Default behavior is having an unknown start position
}

std::optional<Point3LL> PrintOperation::findEndPosition() const
{
    return std::nullopt; // Default behavior is having an unknown end position
}

std::shared_ptr<PrintOperationSequence>
    PrintOperation::findParent(const std::function<bool(const std::shared_ptr<PrintOperationSequence>&)>& search_function, bool warn_not_found) const
{
    if (auto parent_ptr = parent_.lock())
    {
        if (search_function(parent_ptr))
        {
            return parent_ptr;
        }

        return parent_ptr->findParent(search_function, warn_not_found);
    }

    if (warn_not_found)
    {
        spdlog::warn("Couldn't find appropriate parent");
    }

    return nullptr;
}

} // namespace cura
