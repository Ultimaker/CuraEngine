/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "math.h"

namespace cura {

int round_divide(int dividend, int divisor)
{
    return (dividend + divisor / 2) / divisor;
}


}//namespace cura
