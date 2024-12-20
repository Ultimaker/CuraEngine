// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <settings/EnumSettings.h>

#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/PrintPlan.h"

namespace cura
{

enum class EPlatformAdhesion;

class SkirtBrimAppender : public PrintOperationTransformer<PrintPlan>
{
public:
    explicit SkirtBrimAppender() = default;

    void process(PrintPlan* print_plan) override;
};

} // namespace cura