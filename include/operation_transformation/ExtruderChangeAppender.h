// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/PrintPlan.h"

namespace cura
{

class ExtruderChangeAppender : public PrintOperationTransformer<PrintPlan>
{
public:
    explicit ExtruderChangeAppender();

    void process(PrintPlan* print_plan) override;
};

} // namespace cura