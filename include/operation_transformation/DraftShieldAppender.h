// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/PrintPlan.h"

namespace cura
{

class PrintPlan;
class Settings;
enum class PrintFeatureType : uint8_t;

class DraftShieldAppender : public PrintOperationTransformer<PrintPlan>
{
public:
    explicit DraftShieldAppender();

    void process(PrintPlan* print_plan) override;

private:
    static std::optional<LayerIndex> calculateMaxLayer(const PrintPlan* print_plan, const Settings& settings);
};

} // namespace cura