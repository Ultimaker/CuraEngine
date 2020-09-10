//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef CURAENGINE_WALLTOOLPATHS_H
#define CURAENGINE_WALLTOOLPATHS_H

#include <memory>

#include "BeadingStrategy/BeadingStrategyFactory.h"
#include "settings/Settings.h"
#include "utils/ExtrusionLine.h"
#include "utils/polygon.h"

namespace cura
{
class WallToolPaths
{
  public:
    WallToolPaths(const Polygons& outline, coord_t nominal_bead_width, size_t inset_count, const Settings& settings);

    const ToolPaths& generate();

    const Polygons& getInnerContour();

    const Polygons& getOutline() const;

    const BinJunctions& getBinJunctions();

    static BinJunctions toolPathsToBinJunctions(const ToolPaths& toolpaths, coord_t num_insets);

    static Polygons innerContourFromToolpaths(const ToolPaths& toolpaths);

  private:
    const Polygons& outline;
    const coord_t nominal_bead_width;
    const size_t inset_count;
    const StrategyType strategy_type;
    const bool widening_beading_enabled;
    std::unique_ptr<coord_t> min_bead_width;
    std::unique_ptr<coord_t> min_feature_size;
    const double small_area_length;
    const coord_t transition_length;
    bool toolpaths_generated;
    ToolPaths toolpaths;
    Polygons inner_contour;
    BinJunctions binJunctions;


//    /*!
// * Convert the wall_toolpaths generated with libArachne to a bin of WallJunctions such that the are easily added to
// * LayerPlan
// * \param num_insets number of insets
// * \param wall_toolpaths the wall_toolpaths
// * \return a bin of WallJunctions
//     */
//    BinJunctions getBinWallJunctions(size_t num_insets, const ToolPaths& wall_toolpaths);
//
//    void generateWallToolPaths(ToolPaths& wall_toolpaths, const Polygons& outline, const Settings& settings);
};
}

#endif // CURAENGINE_WALLTOOLPATHS_H
