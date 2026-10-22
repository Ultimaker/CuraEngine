// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/DilationKernel.h"

namespace cura
{

DilationKernel::DilationKernel(GridPoint3 kernel_size, DilationKernel::Type type)
    : kernel_size_(kernel_size)
    , type_(type)
{
    coord_t mult = kernel_size.x_ * kernel_size.y_ * kernel_size.z_; // multiplier for division to avoid rounding and to avoid use of floating point numbers
    relative_cells_.reserve(mult);
    GridPoint3 half_kernel = kernel_size / 2;

    GridPoint3 start = -half_kernel;
    GridPoint3 end = kernel_size - half_kernel;
    for (coord_t x = start.x_; x < end.x_; x++)
    {
        for (coord_t y = start.y_; y < end.y_; y++)
        {
            for (coord_t z = start.z_; z < end.z_; z++)
            {
                GridPoint3 current(x, y, z);
                if (type != Type::CUBE)
                {
                    GridPoint3 limit((x < 0) ? start.x_ : end.x_ - 1, (y < 0) ? start.y_ : end.y_ - 1, (z < 0) ? start.z_ : end.z_ - 1);
                    if (limit.x_ == 0)
                        limit.x_ = 1;
                    if (limit.y_ == 0)
                        limit.y_ = 1;
                    if (limit.z_ == 0)
                        limit.z_ = 1;
                    const GridPoint3 rel_dists = mult * current / limit;
                    if ((type == Type::DIAMOND && rel_dists.x_ + rel_dists.y_ + rel_dists.z_ > mult) || (type == Type::PRISM && rel_dists.x_ + rel_dists.y_ > mult))
                    {
                        continue; // don't consider this cell
                    }
                }
                relative_cells_.emplace_back(x, y, z);
            }
        }
    }
}

} // namespace cura
