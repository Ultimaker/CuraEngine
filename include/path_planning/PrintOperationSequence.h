// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_PRINTOPERATIONSEQUENCE_H
#define PATHPLANNING_PRINTOPERATIONSEQUENCE_H

#include "path_planning/PrintOperation.h"

namespace cura
{

class ExtruderMove;
class SliceMeshStorage;
class PathExporter;
class LayerPlan;

class PrintOperationSequence : public PrintOperation
{
public:
    bool empty() const noexcept;

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    virtual void write(PathExporter& exporter, const std::vector<const PrintOperation*>& parents = {}) const override;

protected:
    void appendOperation(const std::shared_ptr<PrintOperation>& operation);

private:
    std::vector<std::shared_ptr<PrintOperation>> operations_;
};

} // namespace cura

#endif // PATHPLANNING_PRINTOPERATIONSEQUENCE_H
