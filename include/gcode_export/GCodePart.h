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

    /*! @return Whether the GCode actually contains print instructions, or management commands (header, heating, custom start gcode, ...) */
    bool isPrintCode() const
    {
        return print_code_;
    }

protected:
    /*!
     * Constructor
     * @param print_code Whether the GCode actually contains print instructions, or management commands (header, heating, custom start gcode, ...)
     */
    explicit GCodePart(const bool print_code = false)
        : print_code_(print_code)
    {
    }

private:
    const bool print_code_; // Whether the GCode actually contains print instructions, or management commands (header, heating, custom start gcode, ...)
};

} // namespace cura

#endif
