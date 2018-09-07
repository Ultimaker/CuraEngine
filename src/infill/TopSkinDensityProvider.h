//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_TOP_SKIN_DENSITY_PROVIDER_H
#define INFILL_TOP_SKIN_DENSITY_PROVIDER_H

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"

namespace cura
{
    
class SliceMeshStorage; // forw decl

class TopSkinDensityProvider : public DensityProvider
{
public:
    TopSkinDensityProvider(const SliceMeshStorage& mesh_data);

    virtual ~TopSkinDensityProvider();

    virtual float operator()(const AABB3D& aabb, const int_fast8_t averaging_statistic) const;

protected:
    AABB3D print_aabb; //!< bounding box of print coordinates in which to apply the image
    const SliceMeshStorage& mesh_data; 

};

} // namespace cura


#endif // INFILL_TOP_SKIN_DENSITY_PROVIDER_H
