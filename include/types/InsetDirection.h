// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * Direction in which to print walls, inside vs. outside.
 */
enum class InsetDirection
{
    /*!
     * The innermost wall is printed first, then the second-innermost wall, etc.
     */
    InsideOut,

    /*!
     * The outermost wall is printed first, then the second wall, etc.
     */
    OutsideIn,

    /*!
     * If the innermost wall is a central wall, it is printed last. Otherwise
     * prints the same as inside out.
     */
    CenterLast,
    Plugin,
};

} // namespace curaengine::types
