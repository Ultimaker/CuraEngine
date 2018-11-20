//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MAPPING_FUNCTION_H
#define UTILS_MAPPING_FUNCTION_H

#include <vector>
#include <functional>

namespace cura
{

/*!
 * Mapping floating input values to floating output values using some arbitrary mapping function
 * 
 * The function consists of a graph of points between which the mapping is interpolated linearly
 * The points on the graph have regular spacing in the X direction so that we can map easily
 * 
 * Outside of the range of points in the graph the nearest graph point will be given
 */
class MappingFunction
{
    using real = float;
    const std::vector<real> points; // !< output reference values
    const real min; //!< input value which outputs to the first output values of \ref points
    const real max; //!< input value which outputs to the last output values of \ref points
    const real step_size; //!< the input size between two consecutive points
public:
    MappingFunction(const std::function<real (real)>& func, real min, real max, uint_fast16_t n_points);
    MappingFunction(const std::vector<real>& points, real min, real max);

    real map(const real in);

    std::function<float (float)> getFunction();
private:
    std::vector<real> evaluateFunction(const std::function<real (real)>& function, real min, real max, uint_fast16_t n_points) const;
};

}//namespace cura
#endif//UTILS_MAPPING_FUNCTION_H

