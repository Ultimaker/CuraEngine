//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_NO_COPY_H
#define UTILS_NO_COPY_H

/*!
 * Util class to base other objects off which should never be copied.
 * Is needed because C++ has an implicit copy constructor and assign operator when none are defined.
*/
class NoCopy
{
protected:
    NoCopy() noexcept
    {
    }
    NoCopy(NoCopy&&) noexcept = default;
    NoCopy& operator=(NoCopy&&) noexcept = default;

    /*!
     * Copy constructor.
     * Cannot be called because it is deleted.
     */
    NoCopy(const NoCopy&) = delete;

    /*!
     * Assign (copy) operator.
     * Cannot be called because it is deleted.
     */
    NoCopy& operator=(const NoCopy&) = delete;
};

#endif // UTILS_NO_COPY_H