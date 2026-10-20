// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{
/*!
 * Direction in which to print walls, inside vs. outside.
 */
enum class InsetDirection
{
    /*!
     * The innermost wall is printed first, then the second-innermost wall, etc.
     */
    INSIDE_OUT,

    /*!
     * The outermost wall is printed first, then the second wall, etc.
     */
    OUTSIDE_IN,

    /*!
     * If the innermost wall is a central wall, it is printed last. Otherwise
     * prints the same as inside out.
     */
    CENTER_LAST,
    PLUGIN,
};

} // namespace cura
