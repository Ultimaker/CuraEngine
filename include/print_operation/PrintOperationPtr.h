// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>

namespace cura
{

class PrintOperation;

using PrintOperationPtr = std::shared_ptr<PrintOperation>;
using ConstPrintOperationPtr = std::shared_ptr<const PrintOperation>;

} // namespace cura
