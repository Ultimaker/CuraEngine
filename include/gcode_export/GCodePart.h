// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODEEXPORT_GCODEPART_H
#define GCODEEXPORT_GCODEPART_H

#include <string>

namespace cura
{

class GCodePart
{
public:
    virtual ~GCodePart() = default;

    virtual std::string str() const = 0;

protected:
    explicit GCodePart() = default;
};

} // namespace cura

#endif
