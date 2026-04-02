// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODEEXPORT_GCODEPART_H
#define GCODEEXPORT_GCODEPART_H

#include <string>

namespace cura
{

/*! \brief Abstract class that is a container for pieces of GCode */
class GCodePart
{
public:
    virtual ~GCodePart() = default;

    /*! \brief Gets the full piece of GCode to be exported */
    virtual std::string str() const = 0;

protected:
    explicit GCodePart() = default;
};

} // namespace cura

#endif
