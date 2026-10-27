// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

/*!
 * Util class to base other objects off which should never be copied.
 * Is needed because C++ has an implicit copy constructor and assign operator when none are defined.
 */
class NoCopy
{
protected:
    NoCopy()
    {
    }

private:
    /*!
     * Private copy constructor.
     * Cannot be called because it is private.
     */
    NoCopy(const NoCopy&);

    /*!
     * Private assign operator.
     * Cannot be called because it is private.
     */
    NoCopy& operator=(const NoCopy&);
};
