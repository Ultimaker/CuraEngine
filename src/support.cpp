// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "support.h"

#include <cmath> // sqrt, round
#include <deque>
#include <fstream> // ifstream.good()
#include <utility> // pair

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/slice.hpp>
#include <range/v3/view/zip.hpp>
#include <scripta/logger.h>
#include <spdlog/spdlog.h>

#include "Application.h" //To get settings.
#include "BoostInterface.hpp"
#include "ExtruderTrain.h"
#include "SkeletalTrapezoidation.h"
#include "Slice.h"
#include "infill.h"
#include "infill/ImageBasedDensityProvider.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/UniformDensityProvider.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h" //For EFillMethod.
#include "settings/types/Angle.h" //To compute overhang distance from the angle.
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "slicer.h"
#include "utils/Simplify.h"
#include "utils/ThreadPool.h"
#include "utils/VoronoiUtils.h"
#include "utils/math.h"
#include "utils/views/get.h"

namespace cura
{

bool AreaSupport::handleSupportModifierMesh(SliceDataStorage& storage, const Settings& mesh_settings, const Slicer* slicer)
{
    if (! mesh_settings.get<bool>("anti_overhang_mesh") && ! mesh_settings.get<bool>("support_mesh"))
    {
        return false;
    }
    enum ModifierType
    {
        ANTI_OVERHANG,
        SUPPORT_DROP_DOWN,
        SUPPORT_VANILLA
    };
    ModifierType modifier_type
        = (mesh_settings.get<bool>("anti_overhang_mesh")) ? ANTI_OVERHANG : ((mesh_settings.get<bool>("support_mesh_drop_down")) ? SUPPORT_DROP_DOWN : SUPPORT_VANILLA);
    for (LayerIndex layer_nr = 0; layer_nr < slicer->layers.size(); layer_nr++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
        const SlicerLayer& slicer_layer = slicer->layers[layer_nr];
        switch (modifier_type)
        {
        case ANTI_OVERHANG:
            support_layer.anti_overhang.add(slicer_layer.polygons);
            break;
        case SUPPORT_DROP_DOWN:
            support_layer.support_mesh_drop_down.add(slicer_layer.polygons);
            break;
        case SUPPORT_VANILLA:
            support_layer.support_mesh.add(slicer_layer.polygons);
            break;
        }
    }
    return true;
}


void AreaSupport::splitGlobalSupportAreasIntoSupportInfillParts(
    SliceDataStorage& storage,
    const std::vector<Polygons>& global_support_areas_per_layer,
    unsigned int total_layer_count)
{
    if (total_layer_count == 0)
    {
        return;
    }

    size_t min_layer = 0;
    size_t max_layer = total_layer_count - 1;

    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const ExtruderTrain& infill_extruder = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const EFillMethod support_pattern = infill_extruder.settings.get<EFillMethod>("support_pattern");
    const coord_t support_line_width = infill_extruder.settings.get<coord_t>("support_line_width");

    // The wall line count is used for calculating insets, and we generate support infill patterns within the insets
    const size_t wall_line_count = infill_extruder.settings.get<size_t>("support_wall_count");

    // Generate separate support islands
    for (LayerIndex layer_nr = 0; layer_nr < total_layer_count - 1; ++layer_nr)
    {
        unsigned int wall_line_count_this_layer = wall_line_count;
        if (layer_nr == 0 && (support_pattern == EFillMethod::LINES || support_pattern == EFillMethod::ZIG_ZAG))
        { // The first layer will be printed with a grid pattern
            wall_line_count_this_layer++;
        }

        const Polygons& global_support_areas = global_support_areas_per_layer[layer_nr];
        if (global_support_areas.size() == 0 || layer_nr < min_layer || layer_nr > max_layer)
        {
            // Initialize support_infill_parts empty
            storage.support.supportLayers[layer_nr].support_infill_parts.clear();
            continue;
        }

        coord_t support_line_width_here = support_line_width;
        if (layer_nr == 0 && mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::RAFT)
        {
            support_line_width_here *= infill_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
        }
        // We don't generate insets and infill area for the parts yet because later the skirt/brim and prime
        // tower will remove themselves from the support, so the outlines of the parts can be changed.
        storage.support.supportLayers[layer_nr].fillInfillParts(layer_nr, global_support_areas_per_layer, support_line_width_here, wall_line_count_this_layer);
    }
}


void AreaSupport::generateSupportInfillFeatures(SliceDataStorage& storage)
{
    AreaSupport::generateGradualSupport(storage);

    // combine support infill layers
    AreaSupport::combineSupportInfillLayers(storage);

    AreaSupport::cleanup(storage);
}

void AreaSupport::generateGradualSupport(SliceDataStorage& storage)
{
    //
    // # How gradual support infill works:
    // The gradual support infill uses the same technic as the gradual infill. Here is an illustration
    //
    //          A part of the model |
    //                              V
    //              __________________________       <-- each gradual support infill step consists of 2 actual layers
    //            / |                         |      <-- each step halfs the infill density.
    //          /_ _ _ _ _ _ _ _ _ _ _ _ _ _ _|      <-- infill density 1 x 1/(2^2) (25%)
    //        / |   |                         |      <-- infill density 1 x 1/(2^1) (50%)
    //      /_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _|
    //    / |   |   |                         |      <-- infill density 1 x 1/(2^0) (100%)
    //  /_____________________________________|      <-- the current layer, the layer which we are processing right now.
    //
    //  | 0 | 1 | 2 |              3          |      <-- areas with different densities, where densities (or sparsities) are represented by numbers 0, 1, 2, etc.
    //  more dense    ---->      less dense
    //
    // In the example above, it shows a part of the model. For the gradual support infill, we process each layer in the following way:
    //  -> Separate the generated support areas into isolated islands, and we process those islands one by one
    //     so that the outlines and the gradual infill areas of an island can be printed together.
    //
    //  -> Calculate a number of layer groups, each group consists of layers that will be infilled with the same density.
    //     The maximum number of densities is defined by the parameter "max graudual support steps". For example, it the <max steps> is 5,
    //     It means that we can have at most 5 different densities, each is 1/2 than the other. So, it will looks like below:
    //           |  step  |    density    |  description              |
    //           |    0   |   1 / (2^0)   |  100% the most dense      |
    //           |    1   |   1 / (2^1)   |                           |
    //           |    2   |   1 / (2^2)   |                           |
    //           |    3   |   1 / (2^3)   |                           |
    //           |    4   |   1 / (2^4)   |                           |
    //           |    5   |   1 / (2^5)   |  3.125% the least dense   |
    //
    //  -> With those layer groups, we can project areas with different densities onto the layer we are currently processing.
    //     Like in the illustration above, you can see 4 different areas with densities ranging from 0 to 3.
    //
    //  -> We save those areas with different densities into the SupportInfillPart data structure, which holds all the support infill
    //     information of a support island on a specific layer.
    //
    //  -> Note that this function only does the above, which is identifying and storing support infill areas with densities.
    //     The actual printing part is done in FffGcodeWriter.
    //
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const size_t total_layer_count = storage.print_layer_count;
    const ExtruderTrain& infill_extruder = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const coord_t gradual_support_step_height = infill_extruder.settings.get<coord_t>("gradual_support_infill_step_height");
    const size_t max_density_steps = infill_extruder.settings.get<size_t>("gradual_support_infill_steps");

    const coord_t wall_count = infill_extruder.settings.get<size_t>("support_wall_count");
    const coord_t wall_width = infill_extruder.settings.get<coord_t>("support_line_width");

    // no early-out for this function; it needs to initialize the [infill_area_per_combine_per_density]
    double layer_skip_count{ 8.0 }; // skip every so many layers as to ignore small gaps in the model making computation more easy
    size_t gradual_support_step_layer_count
        = round_divide(gradual_support_step_height, mesh_group_settings.get<coord_t>("layer_height")); // The difference in layer count between consecutive density infill areas.

    // make gradual_support_step_height divisable by layer_skip_count
    const auto n_skip_steps_per_gradual_step
        = std::max(1.0, std::ceil(gradual_support_step_layer_count / layer_skip_count)); // only decrease layer_skip_count to make it a divisor of gradual_support_step_layer_count
    layer_skip_count = gradual_support_step_layer_count / n_skip_steps_per_gradual_step;

    LayerIndex min_layer = 0;
    LayerIndex max_layer = total_layer_count - 1;

    // compute different density areas for each support island
    for (LayerIndex layer_nr = 0; layer_nr < total_layer_count - 1; layer_nr++)
    {
        if (layer_nr < min_layer || layer_nr > max_layer)
        {
            continue;
        }

        // generate separate support islands and calculate density areas for each island
        std::vector<SupportInfillPart>& support_infill_parts = storage.support.supportLayers[layer_nr].support_infill_parts;
        for (unsigned int part_idx = 0; part_idx < support_infill_parts.size(); ++part_idx)
        {
            SupportInfillPart& support_infill_part = support_infill_parts[part_idx];

            Polygons original_area = support_infill_part.getInfillArea();
            if (original_area.empty())
            {
                continue;
            }
            // NOTE: This both generates the walls _and_ returns the _actual_ infill area (the one _without_ walls) for use in the rest of the method.
            const Polygons infill_area = Infill::generateWallToolPaths(
                support_infill_part.wall_toolpaths,
                original_area,
                support_infill_part.inset_count_to_generate,
                wall_width,
                0,
                infill_extruder.settings,
                layer_nr,
                SectionType::SUPPORT);
            const AABB& this_part_boundary_box = support_infill_part.outline_boundary_box;

            // calculate density areas for this island
            Polygons less_dense_support = infill_area; // one step less dense with each density_step
            for (unsigned int density_step = 0; density_step < max_density_steps; ++density_step)
            {
                LayerIndex min_layer{ layer_nr + density_step * gradual_support_step_layer_count + static_cast<LayerIndex::value_type>(layer_skip_count) };
                LayerIndex max_layer{ layer_nr + (density_step + 1) * gradual_support_step_layer_count };

                for (float upper_layer_idx = min_layer; upper_layer_idx <= max_layer; upper_layer_idx += layer_skip_count)
                {
                    if (static_cast<unsigned int>(upper_layer_idx) >= total_layer_count)
                    {
                        less_dense_support.clear();
                        break;
                    }

                    // compute intersections with relevant upper parts
                    const std::vector<SupportInfillPart> upper_infill_parts = storage.support.supportLayers[upper_layer_idx].support_infill_parts;
                    Polygons relevant_upper_polygons;
                    for (unsigned int upper_part_idx = 0; upper_part_idx < upper_infill_parts.size(); ++upper_part_idx)
                    {
                        if (support_infill_part.outline.empty())
                        {
                            continue;
                        }

                        // we compute intersection based on support infill areas
                        const AABB& upper_part_boundary_box = upper_infill_parts[upper_part_idx].outline_boundary_box;
                        //
                        // Here we are comparing the **outlines** of the infill areas
                        //
                        // legend:
                        //   ^ support roof
                        //   | support wall
                        //   # dense support
                        //   + less dense support
                        //
                        //     comparing infill            comparing with outline (this is our approach)
                        //    ^^^^^^        ^^^^^^            ^^^^^^            ^^^^^^
                        //    ####||^^      ####||^^          ####||^^          ####||^^
                        //    ######||^^    #####||^^         ######||^^        #####||^^
                        //    ++++####||    ++++##||^         ++++++##||        ++++++||^
                        //    ++++++####    +++++##||         ++++++++##        +++++++||
                        //
                        if (upper_part_boundary_box.hit(this_part_boundary_box))
                        {
                            relevant_upper_polygons.add(upper_infill_parts[upper_part_idx].outline);
                        }
                    }

                    less_dense_support = less_dense_support.intersection(relevant_upper_polygons);
                }
                if (less_dense_support.size() == 0)
                {
                    break;
                }

                // add new infill_area_per_combine_per_density for the current density
                support_infill_part.infill_area_per_combine_per_density.emplace_back();
                std::vector<Polygons>& support_area_current_density = support_infill_part.infill_area_per_combine_per_density.back();
                const Polygons more_dense_support = infill_area.difference(less_dense_support);
                support_area_current_density.push_back(more_dense_support);
            }

            support_infill_part.infill_area_per_combine_per_density.emplace_back();
            std::vector<Polygons>& support_area_current_density = support_infill_part.infill_area_per_combine_per_density.back();
            support_area_current_density.push_back(infill_area);

            assert(support_infill_part.infill_area_per_combine_per_density.size() != 0 && "support_infill_part.infill_area_per_combine_per_density should now be initialized");
#ifdef DEBUG
            for (unsigned int part_i = 0; part_i < support_infill_part.infill_area_per_combine_per_density.size(); ++part_i)
            {
                assert(support_infill_part.infill_area_per_combine_per_density[part_i].size() != 0);
            }
#endif // DEBUG
        }
    }
}


void AreaSupport::combineSupportInfillLayers(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const unsigned int total_layer_count = storage.print_layer_count;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    // How many support infill layers to combine to obtain the requested sparse thickness.
    const ExtruderTrain& infill_extruder = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const size_t combine_layers_amount
        = std::max(uint64_t(1), round_divide(infill_extruder.settings.get<coord_t>("support_infill_sparse_thickness"), std::max(layer_height, coord_t(1))));
    if (combine_layers_amount <= 1)
    {
        return;
    }

    /* We need to round down the layer index we start at to the nearest
    divisible index. Otherwise we get some parts that have infill at divisible
    layers and some at non-divisible layers. Those layers would then miss each
    other. */
    size_t min_layer = combine_layers_amount - 1;
    min_layer -= min_layer % combine_layers_amount; // Round upwards to the nearest layer divisible by infill_sparse_combine.
    size_t max_layer = total_layer_count < storage.support.supportLayers.size() ? total_layer_count : storage.support.supportLayers.size();
    max_layer = max_layer - 1;
    max_layer -= max_layer % combine_layers_amount; // Round downwards to the nearest layer divisible by infill_sparse_combine.

    for (size_t layer_idx = min_layer; layer_idx <= max_layer; layer_idx += combine_layers_amount) // Skip every few layers, but extrude more.
    {
        if (layer_idx >= storage.support.supportLayers.size())
        {
            break;
        }

        SupportLayer& layer = storage.support.supportLayers[layer_idx];
        for (unsigned int combine_count_here = 1; combine_count_here < combine_layers_amount; ++combine_count_here)
        {
            if (layer_idx < combine_count_here)
            {
                break;
            }

            size_t lower_layer_idx = layer_idx - combine_count_here;
            if (lower_layer_idx < min_layer)
            {
                break;
            }
            SupportLayer& lower_layer = storage.support.supportLayers[lower_layer_idx];

            for (SupportInfillPart& part : layer.support_infill_parts)
            {
                if (part.getInfillArea().empty())
                {
                    continue;
                }
                for (unsigned int density_idx = 0; density_idx < part.infill_area_per_combine_per_density.size(); ++density_idx)
                { // go over each density of gradual infill (these density areas overlap!)
                    std::vector<Polygons>& infill_area_per_combine = part.infill_area_per_combine_per_density[density_idx];
                    Polygons result;
                    for (SupportInfillPart& lower_layer_part : lower_layer.support_infill_parts)
                    {
                        if (! part.outline_boundary_box.hit(lower_layer_part.outline_boundary_box))
                        {
                            continue;
                        }

                        Polygons intersection = infill_area_per_combine[combine_count_here - 1].intersection(lower_layer_part.getInfillArea()).offset(-200).offset(200);
                        if (intersection.size() <= 0)
                        {
                            continue;
                        }

                        result.add(intersection); // add area to be thickened
                        infill_area_per_combine[combine_count_here - 1]
                            = infill_area_per_combine[combine_count_here - 1].difference(intersection); // remove thickened area from less thick layer here

                        unsigned int max_lower_density_idx = density_idx;
                        // Generally: remove only from *same density* areas on layer below
                        // If there are no same density areas, then it's ok to print them anyway
                        // Don't remove other density areas
                        if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
                        {
                            // For the most dense areas on a given layer the density of that area is doubled.
                            // This means that - if the lower layer has more densities -
                            // all those lower density lines are included in the most dense of this layer.
                            // We therefore compare the most dense are on this layer with all densities
                            // of the lower layer with the same or higher density index
                            max_lower_density_idx = lower_layer_part.infill_area_per_combine_per_density.size() - 1;
                        }
                        for (unsigned int lower_density_idx = density_idx;
                             lower_density_idx <= max_lower_density_idx && lower_density_idx < lower_layer_part.infill_area_per_combine_per_density.size();
                             lower_density_idx++)
                        {
                            std::vector<Polygons>& lower_infill_area_per_combine = lower_layer_part.infill_area_per_combine_per_density[lower_density_idx];
                            lower_infill_area_per_combine[0]
                                = lower_infill_area_per_combine[0].difference(intersection); // remove thickened area from lower (single thickness) layer
                        }
                    }

                    infill_area_per_combine.push_back(result);
                }
            }
        }
    }
}

void AreaSupport::cleanup(SliceDataStorage& storage)
{
    const coord_t support_line_width = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("support_line_width");
    for (LayerIndex layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++)
    {
        SupportLayer& layer = storage.support.supportLayers[layer_nr];
        for (unsigned int part_idx = 0; part_idx < layer.support_infill_parts.size(); part_idx++)
        {
            SupportInfillPart& part = layer.support_infill_parts[part_idx];
            bool can_be_removed = true;
            if (part.inset_count_to_generate > 0)
            {
                can_be_removed = false;
            }
            else
            {
                for (const std::vector<Polygons>& infill_area_per_combine_this_density : part.infill_area_per_combine_per_density)
                {
                    for (const Polygons& infill_area_this_combine_this_density : infill_area_per_combine_this_density)
                    {
                        // remove small areas which were introduced by rounding errors in comparing the same area on two consecutive layer
                        if (! infill_area_this_combine_this_density.empty() && infill_area_this_combine_this_density.area() > support_line_width * support_line_width)
                        {
                            can_be_removed = false;
                            break;
                        }
                    }
                    if (! can_be_removed)
                    { // break outer loop
                        break;
                    }
                }
            }
            if (can_be_removed)
            {
                part = std::move(layer.support_infill_parts.back());
                layer.support_infill_parts.pop_back();
                part_idx--;
            }
        }
    }
}

Polygons AreaSupport::join(const SliceDataStorage& storage, const Polygons& supportLayer_up, Polygons& supportLayer_this, const coord_t smoothing_distance)
{
    Polygons joined;

    const Settings& infill_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr").settings;
    const AngleRadians conical_support_angle = infill_settings.get<AngleRadians>("support_conical_angle");
    const coord_t layer_thickness = infill_settings.get<coord_t>("layer_height");
    coord_t conical_support_offset;
    if (conical_support_angle > 0)
    { // outward ==> wider base than overhang
        conical_support_offset = -(tan(conical_support_angle) - 0.01) * layer_thickness;
    }
    else
    { // inward ==> smaller base than overhang
        conical_support_offset = (tan(-conical_support_angle) - 0.01) * layer_thickness;
    }
    const bool conical_support = infill_settings.get<bool>("support_conical_enabled") && conical_support_angle != 0;
    if (conical_support)
    {
        const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
        // Don't go outside the build volume.
        Polygons machine_volume_border;
        switch (mesh_group_settings.get<BuildPlateShape>("machine_shape"))
        {
        case BuildPlateShape::ELLIPTIC:
        {
            // Construct an ellipse to approximate the build volume.
            const coord_t width = storage.machine_size.max.x - storage.machine_size.min.x;
            const coord_t depth = storage.machine_size.max.y - storage.machine_size.min.y;
            Polygon border_circle;
            constexpr unsigned int circle_resolution = 50;
            for (unsigned int i = 0; i < circle_resolution; i++)
            {
                const AngleRadians angle = TAU * i / circle_resolution;
                const Point3 machine_middle = storage.machine_size.getMiddle();
                const coord_t x = machine_middle.x + cos(angle) * width / 2;
                const coord_t y = machine_middle.y + sin(angle) * depth / 2;
                border_circle.emplace_back(x, y);
            }
            machine_volume_border.add(border_circle);
            break;
        }
        case BuildPlateShape::RECTANGULAR:
        default:
            machine_volume_border.add(storage.machine_size.flatten().toPolygon());
            break;
        }
        coord_t adhesion_size = 0; // Make sure there is enough room for the platform adhesion around support.
        coord_t extra_skirt_line_width = 0;
        const std::vector<bool> is_extruder_used = storage.getExtrudersUsed();
        for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
        {
            if (! is_extruder_used[extruder_nr]) // Unused extruders and the primary adhesion extruder don't generate an extra skirt line.
            {
                continue;
            }
            const ExtruderTrain& other_extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];
            extra_skirt_line_width += other_extruder.settings.get<coord_t>("skirt_brim_line_width") * other_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
        }
        const std::vector<ExtruderTrain*> skirt_brim_extruders = mesh_group_settings.get<std::vector<ExtruderTrain*>>("skirt_brim_extruder_nr");
        auto adhesion_width_str{ "brim_width" };
        auto adhesion_line_count_str{ "brim_line_count" };
        switch (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type"))
        {
        case EPlatformAdhesion::SKIRT:
            adhesion_width_str = "skirt_gap";
            adhesion_line_count_str = "skirt_line_count";
            [[fallthrough]];
        case EPlatformAdhesion::BRIM:
            for (ExtruderTrain* skirt_brim_extruder_p : skirt_brim_extruders)
            {
                ExtruderTrain& skirt_brim_extruder = *skirt_brim_extruder_p;
                adhesion_size = std::max(
                    adhesion_size,
                    coord_t(
                        skirt_brim_extruder.settings.get<coord_t>(adhesion_width_str)
                        + skirt_brim_extruder.settings.get<coord_t>("skirt_brim_line_width")
                              * (skirt_brim_extruder.settings.get<size_t>(adhesion_line_count_str) - 1) // - 1 because the line is also included in extra_skirt_line_width
                              * skirt_brim_extruder.settings.get<Ratio>("initial_layer_line_width_factor")
                        + extra_skirt_line_width));
            }
            break;
        case EPlatformAdhesion::RAFT:
        {
            adhesion_size = std::max({ mesh_group_settings.get<ExtruderTrain&>("raft_base_extruder_nr").settings.get<coord_t>("raft_margin"),
                                       mesh_group_settings.get<ExtruderTrain&>("raft_interface_extruder_nr").settings.get<coord_t>("raft_margin"),
                                       mesh_group_settings.get<ExtruderTrain&>("raft_surface_extruder_nr").settings.get<coord_t>("raft_margin") });
            break;
        }
        case EPlatformAdhesion::NONE:
            adhesion_size = 0;
            break;
        default: // Also use 0.
            spdlog::info("Unknown platform adhesion type! Please implement the width of the platform adhesion here.");
            break;
        }
        machine_volume_border = machine_volume_border.offset(-adhesion_size);

        const coord_t conical_smallest_breadth = infill_settings.get<coord_t>("support_conical_min_width");
        Polygons insetted = supportLayer_up.offset(-conical_smallest_breadth / 2);
        Polygons small_parts = supportLayer_up.difference(insetted.offset(conical_smallest_breadth / 2 + 20));
        joined = supportLayer_this.unionPolygons(supportLayer_up.offset(conical_support_offset)).unionPolygons(small_parts).intersection(machine_volume_border);
    }
    else
    {
        joined = supportLayer_this.unionPolygons(supportLayer_up);
    }

    // join different parts
    const coord_t join_distance = infill_settings.get<coord_t>("support_join_distance");
    if (join_distance > 0)
    {
        // first offset the layer a little inwards; this way tiny area's will not be joined
        // (narrow areas; ergo small areas will be removed in a later step using this same offset)
        // this inwards offset is later reversed by increasing the outwards offset
        const coord_t join_distance = infill_settings.get<coord_t>("support_join_distance");
        const coord_t support_line_width = infill_settings.get<coord_t>("support_line_width");
        const coord_t min_even_wall_line_width = infill_settings.get<coord_t>("min_even_wall_line_width");
        auto half_min_feature_width = min_even_wall_line_width + 10;

        joined = joined.offset(-half_min_feature_width)
                     .offset(join_distance + half_min_feature_width, ClipperLib::jtRound)
                     .offset(-join_distance, ClipperLib::jtRound)
                     .unionPolygons(joined);
    }

    const Simplify simplify(infill_settings);
    joined = simplify.polygon(joined);

    return joined;
}

void AreaSupport::generateOverhangAreas(SliceDataStorage& storage)
{
    for (std::shared_ptr<SliceMeshStorage>& mesh_ptr : storage.meshes)
    {
        auto& mesh = *mesh_ptr;
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            continue;
        }
        // it actually also initializes some buffers that are needed in generateSupport
        generateOverhangAreasForMesh(storage, mesh);
    }
}

void AreaSupport::generateSupportAreas(SliceDataStorage& storage)
{
    std::vector<Polygons> global_support_areas_per_layer;
    global_support_areas_per_layer.resize(storage.print_layer_count);

    int max_layer_nr_support_mesh_filled;
    for (max_layer_nr_support_mesh_filled = storage.support.supportLayers.size() - 1; max_layer_nr_support_mesh_filled >= 0; max_layer_nr_support_mesh_filled--)
    {
        const SupportLayer& support_layer = storage.support.supportLayers[max_layer_nr_support_mesh_filled];
        if (support_layer.support_mesh.size() > 0 || support_layer.support_mesh_drop_down.size() > 0)
        {
            break;
        }
    }
    storage.support.layer_nr_max_filled_layer = max_layer_nr_support_mesh_filled;
    for (int layer_nr = 0; layer_nr < max_layer_nr_support_mesh_filled; layer_nr++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
        support_layer.anti_overhang = support_layer.anti_overhang.unionPolygons();
        support_layer.support_mesh_drop_down = support_layer.support_mesh_drop_down.unionPolygons();
        support_layer.support_mesh = support_layer.support_mesh.unionPolygons();
    }

    // initialization of supportAreasPerLayer
    if (storage.print_layer_count > storage.support.supportLayers.size())
    { // there might already be anti_overhang_area data or support mesh data in the supportLayers
        storage.support.supportLayers.resize(storage.print_layer_count);
    }

    // generate support areas
    bool support_meshes_drop_down_handled = false;
    bool support_meshes_handled = false;
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
        if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
        {
            continue;
        }
        Settings* infill_settings = &storage.meshes[mesh_idx]->settings;
        Settings* roof_settings = &storage.meshes[mesh_idx]->settings;
        Settings* bottom_settings = &storage.meshes[mesh_idx]->settings;
        if (mesh.settings.get<bool>("support_mesh"))
        {
            if ((mesh.settings.get<bool>("support_mesh_drop_down") && support_meshes_drop_down_handled)
                || (! mesh.settings.get<bool>("support_mesh_drop_down") && support_meshes_handled))
            { // handle all support_mesh and support_mesh_drop_down areas only once
                continue;
            }
            // use extruder train settings rather than the per-object settings of the first support mesh encountered.
            // because all support meshes are processed at the same time it doesn't make sense to use the per-object settings of the first support mesh encountered.
            // instead we must use the support extruder settings, which is the settings base common to all support meshes.
            infill_settings = &mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").settings;
            roof_settings = &mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").settings;
            bottom_settings = &mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").settings;
            if (mesh.settings.get<bool>("support_mesh_drop_down"))
            {
                support_meshes_drop_down_handled = true;
            }
            else
            {
                support_meshes_handled = true;
            }
        }
        std::vector<Polygons> mesh_support_areas_per_layer;
        mesh_support_areas_per_layer.resize(storage.print_layer_count, Polygons());

        generateSupportAreasForMesh(storage, *infill_settings, *roof_settings, *bottom_settings, mesh_idx, storage.print_layer_count, mesh_support_areas_per_layer);
        for (size_t layer_idx = 0; layer_idx < storage.print_layer_count; layer_idx++)
        {
            global_support_areas_per_layer[layer_idx].add(mesh_support_areas_per_layer[layer_idx]);
        }
    }

    for (Polygons& support_areas : global_support_areas_per_layer)
    {
        support_areas = support_areas.unionPolygons();
    }

    // handle support interface
    for (auto& mesh : storage.meshes)
    {
        if (mesh->settings.get<bool>("infill_mesh") || mesh->settings.get<bool>("anti_overhang_mesh"))
        {
            continue;
        }

        if (mesh->settings.get<bool>("support_roof_enable"))
        {
            generateSupportRoof(storage, *mesh, global_support_areas_per_layer);
        }
        if (mesh->settings.get<bool>("support_bottom_enable"))
        {
            generateSupportBottom(storage, *mesh, global_support_areas_per_layer);
        }
    }

    // split the global support areas into parts for later gradual support infill generation
    AreaSupport::splitGlobalSupportAreasIntoSupportInfillParts(storage, global_support_areas_per_layer, storage.print_layer_count);
    precomputeCrossInfillTree(storage);
}

void AreaSupport::precomputeCrossInfillTree(SliceDataStorage& storage)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const ExtruderTrain& infill_extruder = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const EFillMethod& support_pattern = infill_extruder.settings.get<EFillMethod>("support_pattern");
    if ((support_pattern == EFillMethod::CROSS || support_pattern == EFillMethod::CROSS_3D) && infill_extruder.settings.get<coord_t>("support_line_distance") > 0)
    {
        AABB3D aabb;
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            const SliceMeshStorage& mesh = *storage.meshes[mesh_idx];
            if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
            {
                continue;
            }
            Settings& infill_settings = storage.meshes[mesh_idx]->settings;
            if (mesh.settings.get<bool>("support_mesh"))
            {
                // use extruder train settings rather than the per-object settings of the first support mesh encountered.
                // because all support meshes are processed at the same time it doesn't make sense to use the per-object settings of the first support mesh encountered.
                // instead we must use the support extruder settings, which is the settings base common to all support meshes.
                infill_settings = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").settings;
            }
            const coord_t aabb_expansion = infill_settings.get<coord_t>("support_offset");
            AABB3D aabb_here(mesh.bounding_box);
            aabb_here.include(aabb_here.min - Point3(-aabb_expansion, -aabb_expansion, 0));
            aabb_here.include(aabb_here.max + Point3(-aabb_expansion, -aabb_expansion, 0));
            aabb.include(aabb_here);
        }

        std::string cross_subdisivion_spec_image_file = infill_extruder.settings.get<std::string>("cross_support_density_image");
        std::ifstream cross_fs(cross_subdisivion_spec_image_file.c_str());
        if (cross_subdisivion_spec_image_file != "" && cross_fs.good())
        {
            storage.support.cross_fill_provider = std::make_shared<SierpinskiFillProvider>(
                aabb,
                infill_extruder.settings.get<coord_t>("support_line_distance"),
                infill_extruder.settings.get<coord_t>("support_line_width"),
                cross_subdisivion_spec_image_file);
        }
        else
        {
            if (cross_subdisivion_spec_image_file != "")
            {
                spdlog::error("Cannot find density image: {}.", cross_subdisivion_spec_image_file);
            }
            storage.support.cross_fill_provider = std::make_shared<SierpinskiFillProvider>(
                aabb,
                infill_extruder.settings.get<coord_t>("support_line_distance"),
                infill_extruder.settings.get<coord_t>("support_line_width"));
        }
    }
}

void AreaSupport::generateOverhangAreasForMesh(SliceDataStorage& storage, SliceMeshStorage& mesh)
{
    if (! mesh.settings.get<bool>("support_enable") && ! mesh.settings.get<bool>("support_mesh"))
    {
        return;
    }

    // Fill the overhang areas with emptiness first, even if it's a support mesh, so that we can request the areas.
    mesh.full_overhang_areas.resize(storage.print_layer_count);
    for (size_t layer_idx = 0; layer_idx < storage.print_layer_count; layer_idx++)
    {
        mesh.overhang_areas.emplace_back();
    }

    // Don't generate overhang areas for support meshes. They are dropped down automatically if desired.
    const bool is_support_modifier = mesh.settings.get<bool>("support_mesh");
    if (is_support_modifier)
    {
        return;
    }

    // Don't generate overhang areas if the Z distance is higher than the objects we're generating support for.
    const coord_t layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
    const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
    const size_t z_distance_top_layers = (z_distance_top / layer_height) + 1;
    if (z_distance_top_layers + 1 > storage.print_layer_count)
    {
        return;
    }

    // Generate points and lines of overhang (for corners pointing downwards, since they don't have an area to support but still need supporting).
    const coord_t max_supported_diameter = mesh.settings.get<coord_t>("support_tower_maximum_supported_diameter");
    const bool use_towers = mesh.settings.get<bool>("support_use_towers") && max_supported_diameter > 0;
    if (use_towers)
    {
        AreaSupport::detectOverhangPoints(storage, mesh);
    }

    // Generate the actual areas and store them in the mesh.
    cura::parallel_for<size_t>(
        1,
        storage.print_layer_count,
        [&](const size_t layer_idx)
        {
            std::pair<Polygons, Polygons> basic_and_full_overhang = computeBasicAndFullOverhang(storage, mesh, layer_idx);
            mesh.overhang_areas[layer_idx] = basic_and_full_overhang.first; // Store the results.
            mesh.full_overhang_areas[layer_idx] = basic_and_full_overhang.second;
            scripta::log("support_basic_overhang_area", basic_and_full_overhang.first, SectionType::SUPPORT, layer_idx);
            scripta::log("support_full_overhang_area", basic_and_full_overhang.second, SectionType::SUPPORT, layer_idx);
        });
}

Polygons AreaSupport::generateVaryingXYDisallowedArea(const SliceMeshStorage& storage, const Settings& infill_settings, const LayerIndex layer_idx)
{
    const auto& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const Simplify simplify{ mesh_group_settings };
    const auto layer_thickness = mesh_group_settings.get<coord_t>("layer_height");
    const auto support_distance_top = static_cast<double>(mesh_group_settings.get<coord_t>("support_top_distance"));
    const auto support_distance_bot = static_cast<double>(mesh_group_settings.get<coord_t>("support_bottom_distance"));
    const auto overhang_angle = mesh_group_settings.get<AngleRadians>("support_angle");
    const auto xy_distance = static_cast<double>(mesh_group_settings.get<coord_t>("support_xy_distance"));
    const auto xy_distance_overhang = infill_settings.get<coord_t>("support_xy_distance_overhang");

    constexpr coord_t snap_radius = 10;
    constexpr coord_t close_dist = snap_radius + 5; // needs to be larger than the snap radius!
    constexpr coord_t search_radius = 0;

    auto layer_current = simplify.polygon(storage.layers[layer_idx].getOutlines().offset(-close_dist).offset(close_dist));

    // sparse grid for storing the offset distances at each point. For each point there can be multiple offset
    // values as multiple may be calculated when multiple layers are used for z-smoothing of the offsets.
    // The average of all offset dists is taken for the used varying offset. To account for this the commutative
    // offset, and the number of offsets $n$ are stored simultaneously. The final offset used is then commutative
    // equal to commutative_offset / n.
    using point_pair_t = std::pair<size_t, double>;
    using grid_t = SparsePointGridInclusive<point_pair_t>;
    grid_t offset_dist_at_point{ snap_radius };

    // Collection of the various areas we used to calculate the areas for. This is a combination
    //  - the support distance (this is the support top distance for overhang areas, and support
    //    bottom thickness for sloped areas)
    //  - of the delta z between the current layer and layer below (this can vary between the areas
    //    when we use multiple layers for z-smoothing)
    //  - the polygon delta; the xy-distance is calculated separately for overhang and sloped areas.
    //    here either the slope or overhang area is stored
    std::vector<std::tuple<double, double, Polygons>> z_distances_layer_deltas;

    constexpr LayerIndex layer_index_offset{ 1 };

    const LayerIndex layer_idx_below{ std::max(LayerIndex{ layer_idx - layer_index_offset }, LayerIndex{ 0 }) };
    if (layer_idx_below != layer_idx)
    {
        auto layer_below = simplify.polygon(storage.layers[layer_idx_below].getOutlines().offset(-close_dist).offset(close_dist));

        z_distances_layer_deltas.emplace_back(support_distance_top, static_cast<double>(layer_index_offset * layer_thickness), layer_current.difference(layer_below));

        z_distances_layer_deltas.emplace_back(support_distance_bot, static_cast<double>(layer_index_offset * layer_thickness), layer_below.difference(layer_current));
    }

    const LayerIndex layer_idx_above{ std::min(LayerIndex{ layer_idx + layer_index_offset }, LayerIndex{ storage.layers.size() - 1 }) };
    if (layer_idx_above != layer_idx)
    {
        auto layer_above = simplify.polygon(storage.layers[layer_idx_below].getOutlines().offset(-close_dist).offset(close_dist));

        z_distances_layer_deltas.emplace_back(support_distance_bot, static_cast<double>(layer_index_offset * layer_thickness), layer_current.difference(layer_above));

        z_distances_layer_deltas.emplace_back(support_distance_top, static_cast<double>(layer_index_offset * layer_thickness), layer_above.difference(layer_current));
    }

    for (auto& [support_distance, delta_z, layer_delta_] : z_distances_layer_deltas)
    {
        const auto xy_distance_natural = support_distance * std::tan(overhang_angle);

        // perform a close operation to remove narrow areas; these cannot easily be turned into a voronoi diagram
        // we might "miss" some vertices in the resulting git map, this is not a problem
        auto layer_delta = layer_delta_.offset(-close_dist).offset(close_dist);

        if (layer_delta.empty())
        {
            continue;
        }

        // grid for storing the "slope" (wall overhang area at that specific point in the polygon)
        grid_t slope_at_point{ snap_radius };

        // construct a voronoi diagram. The slope is calculated based
        // on the edge length from the boundary to the center edge(s)
        std::vector<SkeletalTrapezoidation::Segment> segments;
        for (auto [poly_idx, poly] : layer_delta | ranges::views::enumerate)
        {
            for (auto [point_idx, _p] : poly | ranges::views::enumerate)
            {
                segments.emplace_back(&layer_delta, poly_idx, point_idx);
            }
        }

        boost::polygon::voronoi_diagram<double> vonoroi_diagram;
        boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vonoroi_diagram);

        for (const auto& edge : vonoroi_diagram.edges())
        {
            if (edge.is_infinite())
            {
                continue;
            }

            auto p0 = VoronoiUtils::p(edge.vertex0());
            auto p1 = VoronoiUtils::p(edge.vertex1());

            // skip edges that move "outside" the polygon;
            // these are st edges that are inside polygon-holes
            if (! layer_delta.inside(p0) && ! layer_delta.inside(p1))
            {
                continue;
            }

            auto dist_to_center_edge = static_cast<double>(cura::vSize(p0 - p1));

            if (dist_to_center_edge < snap_radius)
            {
                continue;
            }

            // p0 to p1 is the distance to the center between the two polygons; two times
            // this distance is (approximately) the distance between the boundaries
            auto dist_to_boundary = 2. * dist_to_center_edge;
            auto slope = dist_to_boundary / delta_z;

            auto nearby_vals = slope_at_point.getNearbyVals(p0, search_radius);
            auto n = ranges::accumulate(nearby_vals | views::get(&point_pair_t::first), 0);
            auto cumulative_slope = ranges::accumulate(nearby_vals | views::get(&point_pair_t::second), 0.);

            n += 1;
            cumulative_slope += slope;

            // update cumulative_slope in sparse grid
            slope_at_point.insert(p0, { n, cumulative_slope });
        }

        for (const auto& poly : layer_current)
        {
            for (const auto& point : poly)
            {
                auto nearby_vals = slope_at_point.getNearbyVals(point, search_radius);
                auto n = ranges::accumulate(nearby_vals | views::get(&point_pair_t::first), 0);
                auto cumulative_slope = ranges::accumulate(nearby_vals | views::get(&point_pair_t::second), 0.);

                if (n != 0)
                {
                    auto slope = cumulative_slope / static_cast<double>(n);
                    auto wall_angle = std::atan(slope);
                    auto ratio = std::min(wall_angle / overhang_angle, 1.);

                    auto xy_distance_varying = std::lerp(xy_distance, xy_distance_natural, ratio);

                    auto nearby_vals_offset_dist = offset_dist_at_point.getNearbyVals(point, search_radius);

                    // update and insert cumulative varying xy distance in one go
                    offset_dist_at_point.insert(
                        point,
                        { ranges::accumulate(nearby_vals_offset_dist | views::get(&point_pair_t::first), 0) + 1,
                          ranges::accumulate(nearby_vals_offset_dist | views::get(&point_pair_t::second), 0.) + xy_distance_varying });
                }
            }
        }
    }

    std::vector<coord_t> varying_offsets;
    for (const auto& poly : layer_current)
    {
        for (const auto& point : poly)
        {
            auto nearby_vals = offset_dist_at_point.getNearbyVals(point, search_radius);

            auto n = ranges::accumulate(nearby_vals | views::get(&point_pair_t::first), 0);
            auto cumulative_offset_dist = ranges::accumulate(nearby_vals | views::get(&point_pair_t::second), 0.);

            double offset_dist{};
            if (n == 0)
            {
                // if there are no offset dists generated for a vertex $p$ this must mean that vertex $p$ was not
                // present in any of the delta areas. This can only happen if the areas for the current layer and
                // the layer(s) below are perfectly aligned at vertex $p$; the walls at vertex $p$ are vertical.
                // As the wall is vertical the xy_distance is taken at vertex $p$.
                offset_dist = xy_distance;
            }
            else
            {
                auto avg_offset_dist = cumulative_offset_dist / static_cast<double>(n);
                offset_dist = avg_offset_dist;
            }

            varying_offsets.push_back(static_cast<coord_t>(offset_dist));
        }
    }

    const auto smooth_dist = xy_distance / 2.0;
    Polygons varying_xy_disallowed_areas = layer_current
                                               // offset using the varying offset distances we calculated previously
                                               .offset(varying_offsets)
                                               // close operation to smooth the x/y disallowed area boundary. With varying xy distances we see some jumps in the boundary.
                                               // As the x/y disallowed areas "cut in" to support the xy-disallowed area may propagate through the support area. If the
                                               // x/y disallowed area is not smoothed boost has trouble generating a voronoi diagram.
                                               .offset(smooth_dist)
                                               .offset(-smooth_dist);
    scripta::log("support_varying_xy_disallowed_areas", varying_xy_disallowed_areas, SectionType::SUPPORT, layer_idx);
    return varying_xy_disallowed_areas;
}

/*
 * Algorithm:
 * From top layer to bottom layer:
 * - find overhang by looking at the difference between two consecutive layers
 * - join with support areas from layer above
 * - subtract current layer
 * - use the result for the next lower support layer (without doing XY-distance and Z bottom distance, so that a single support beam may move around the model a bit => more
 * stability)
 * - perform inset using X/Y-distance and bottom Z distance
 *
 * for support buildplate only: purge all support not connected to build plate
 */
void AreaSupport::generateSupportAreasForMesh(
    SliceDataStorage& storage,
    const Settings& infill_settings,
    const Settings& roof_settings,
    const Settings& bottom_settings,
    const size_t mesh_idx,
    const size_t layer_count,
    std::vector<Polygons>& support_areas)
{
    SliceMeshStorage& mesh = *storage.meshes[mesh_idx];

    const ESupportStructure support_structure = mesh.settings.get<ESupportStructure>("support_structure");
    const bool is_support_mesh_place_holder
        = mesh.settings.get<bool>("support_mesh"); // whether this mesh has empty SliceMeshStorage and this function is now called to only generate support for all support meshes
    if ((! mesh.settings.get<bool>("support_enable") || support_structure != ESupportStructure::NORMAL) && ! is_support_mesh_place_holder)
    {
        return;
    }
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const ESupportType support_type = mesh_group_settings.get<ESupportType>("support_type");
    if (support_type == ESupportType::NONE && ! is_support_mesh_place_holder)
    {
        return;
    }

    // early out
    const coord_t layer_thickness = mesh_group_settings.get<coord_t>("layer_height");
    const coord_t z_distance_top = ((mesh.settings.get<bool>("support_roof_enable")) ? roof_settings : infill_settings).get<coord_t>("support_top_distance");
    const size_t layer_z_distance_top = (z_distance_top / layer_thickness) + 1;
    if (layer_z_distance_top + 1 > layer_count)
    {
        return;
    }

    // Compute the areas that are disallowed by the X/Y distance.
    std::vector<Polygons> xy_disallowed_per_layer;
    xy_disallowed_per_layer.resize(layer_count);
    std::vector<Polygons> sloped_areas_per_layer;
    sloped_areas_per_layer.resize(layer_count);
    sloped_areas_per_layer[0] = Polygons();
    // simplified processing for bottom layer - just ensure support clears part by XY distance
    const coord_t xy_distance = infill_settings.get<coord_t>("support_xy_distance");
    const coord_t xy_distance_overhang = infill_settings.get<coord_t>("support_xy_distance_overhang");
    const bool use_xy_distance_overhang
        = infill_settings.get<SupportDistPriority>("support_xy_overrides_z") == SupportDistPriority::Z_OVERRIDES_XY; // whether to use a different xy distance at overhangs
    const AngleRadians angle = ((mesh.settings.get<bool>("support_roof_enable")) ? roof_settings : infill_settings).get<AngleRadians>("support_angle");
    const double tan_angle = tan(angle) - 0.01; // the XY-component of the supportAngle
    constexpr bool no_support = false;
    constexpr bool no_prime_tower = false;
    const coord_t support_line_width = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr").settings.get<coord_t>("support_line_width");
    const double sloped_areas_angle = mesh.settings.get<AngleRadians>("support_bottom_stair_step_min_slope");
    const coord_t sloped_area_detection_width = 10 + static_cast<coord_t>(layer_thickness / std::tan(sloped_areas_angle)) / 2;
    const double minimum_support_area = mesh.settings.get<double>("minimum_support_area");
    const coord_t min_even_wall_line_width = mesh.settings.get<coord_t>("min_even_wall_line_width");
    xy_disallowed_per_layer[0] = storage.getLayerOutlines(0, no_support, no_prime_tower).offset(xy_distance);

    // The maximum width of an odd wall = 2 * minimum even wall width.
    auto half_min_feature_width = min_even_wall_line_width + 10;

    cura::parallel_for<size_t>(
        1,
        layer_count,
        [&](const size_t layer_idx)
        {
            const Polygons outlines = storage.getLayerOutlines(layer_idx, no_support, no_prime_tower);

            // Build sloped areas. We need this for the stair-stepping later on.
            // Specifically, sloped areass are used in 'moveUpFromModel' to prevent a stair step happening over an area where there isn't a slope.
            // This part here only concerns the slope between two layers. This will be post-processed later on (see the other parallel loop below).
            sloped_areas_per_layer[layer_idx] =
                // Take the outer areas of the previous layer, where the outer areas are (mostly) just _inside_ the shape.
                storage.getLayerOutlines(layer_idx - 1, no_support, no_prime_tower)
                    .tubeShape(sloped_area_detection_width, 10)
                    // Intersect those with the outer areas of the current layer, where the outer areas are (mostly) _outside_ the shape.
                    // This will detect every slope (and some/most vertical walls) between those two layers.
                    .intersection(outlines.tubeShape(10, sloped_area_detection_width))
                    // Do an opening operation so we're not stuck with tiny patches.
                    // The later offset is extended with the line-width, so all patches are merged together if there's less than a line-width between them.
                    .offset(-10)
                    .offset(10 + sloped_area_detection_width);
            // The sloped areas are now ready to be post-processed.
            scripta::log(
                "support_sloped_areas",
                sloped_areas_per_layer[layer_idx],
                SectionType::SUPPORT,
                layer_idx,
                scripta::CellVDI{ "sloped_area_detection_width", sloped_area_detection_width });

            if (! is_support_mesh_place_holder)
            { // don't compute overhang for support meshes
                if (use_xy_distance_overhang) // Z overrides XY distance.
                {
                    // we also want to use the min XY distance when the support is resting on a sloped surface so we calculate the area of the
                    // layer below that protrudes beyond the current layer's area and combine it with the current layer's overhang disallowed area

                    Polygons minimum_xy_disallowed_areas = xy_disallowed_per_layer[layer_idx].offset(xy_distance_overhang);
                    Polygons varying_xy_disallowed_areas = generateVaryingXYDisallowedArea(mesh, infill_settings, layer_idx);
                    xy_disallowed_per_layer[layer_idx] = minimum_xy_disallowed_areas.unionPolygons(varying_xy_disallowed_areas);
                    scripta::log("support_xy_disallowed_areas", xy_disallowed_per_layer[layer_idx], SectionType::SUPPORT, layer_idx);
                }
            }
            if (is_support_mesh_place_holder || ! use_xy_distance_overhang)
            {
                xy_disallowed_per_layer[layer_idx] = outlines.offset(xy_distance);
            }
        });

    std::vector<Polygons> tower_roofs;
    Polygons stair_removal; // polygons to subtract from support because of stair-stepping

    const bool is_support_mesh_nondrop_place_holder = is_support_mesh_place_holder && ! mesh.settings.get<bool>("support_mesh_drop_down");
    const bool is_support_mesh_drop_down_place_holder = is_support_mesh_place_holder && mesh.settings.get<bool>("support_mesh_drop_down");

    const coord_t bottom_stair_step_width = std::max(static_cast<coord_t>(0), mesh.settings.get<coord_t>("support_bottom_stair_step_width"));
    const coord_t extension_offset = infill_settings.get<coord_t>("support_offset");

    const coord_t max_tower_supported_diameter = infill_settings.get<coord_t>("support_tower_maximum_supported_diameter");
    const bool use_towers = infill_settings.get<bool>("support_use_towers") && max_tower_supported_diameter > 0;

    coord_t smoothing_distance;
    { // compute best smoothing_distance
        const ExtruderTrain& infill_train = mesh_group_settings.get<ExtruderTrain&>("support_infill_extruder_nr");
        const coord_t infill_line_width = infill_train.settings.get<coord_t>("support_line_width");
        smoothing_distance = infill_line_width;
        if (mesh.settings.get<bool>("support_roof_enable"))
        {
            const ExtruderTrain& roof_train = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr");
            const coord_t roof_line_width = roof_train.settings.get<coord_t>("support_roof_line_width");
            smoothing_distance = std::max(smoothing_distance, roof_line_width);
        }

        if (mesh.settings.get<bool>("support_bottom_enable"))
        {
            const ExtruderTrain& bottom_train = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr");
            const coord_t bottom_line_width = bottom_train.settings.get<coord_t>("support_bottom_line_width");
            smoothing_distance = std::max(smoothing_distance, bottom_line_width);
        }
    }

    const coord_t z_distance_bottom = ((mesh.settings.get<bool>("support_bottom_enable")) ? bottom_settings : infill_settings).get<coord_t>("support_bottom_distance");
    const size_t bottom_empty_layer_count = round_up_divide(z_distance_bottom, layer_thickness); // number of empty layers between support and model
    const coord_t bottom_stair_step_height = std::max(static_cast<coord_t>(0), mesh.settings.get<coord_t>("support_bottom_stair_step_height"));
    const size_t bottom_stair_step_layer_count
        = bottom_stair_step_height / layer_thickness + 1; // the difference in layers between two stair steps. One is normal support (not stair-like)

    // Post-process the sloped areas's. (Skip if no stair-stepping anyway.)
    // The idea here is to 'add up' all the sloped 'areas' so they form actual areas per each stair-step height.
    // (Only the 'top' sloped area for each step is actually used in the end, see 'moveUpFromModel'.)
    if (bottom_stair_step_layer_count > 1)
    {
        // We can parallelize this part, which is needed since these are potentially expensive operations,
        // but only in chunks of `bottom_stair_step_layer_count` steps, since, within such a chunk,
        // the order of execution is important.
        // Unless thinking about optimization & threading, you can just think of this as a single for-loop.
        cura::parallel_for<size_t>(
            1,
            layer_count,
            [&](const size_t layer_idx)
            {
                // Add the sloped areas together for each stair of the stair stepping.
                // Start a new stair every modulo bottom_stair_step_layer_count steps.
                if (layer_idx % bottom_stair_step_layer_count != 1)
                {
                    sloped_areas_per_layer[layer_idx] = sloped_areas_per_layer[layer_idx].unionPolygons(sloped_areas_per_layer[layer_idx - 1]);
                }
            },
            bottom_stair_step_layer_count);
    }

    for (size_t layer_idx = layer_count - 1 - layer_z_distance_top; layer_idx != static_cast<size_t>(-1); layer_idx--)
    {
        Polygons layer_this = mesh.full_overhang_areas[layer_idx + layer_z_distance_top];

        if (extension_offset && ! is_support_mesh_place_holder)
        {
            // To avoid that the support is folding around the model, the support horizontal expansion should not cause
            // the support to grow towards the model. Stepwise applying the support horizontal expansion to both the
            // model outline and the support is effectively calculating a voronoi. The offset is first applied to
            // the support and next to the model to ensure that the expanded support area is connected to the original
            // support area. Please note that the horizontal expansion is rounded down to an integer offset_per_step.
            Polygons model_outline = storage.getLayerOutlines(layer_idx, no_support, no_prime_tower);
            const coord_t offset_per_step = support_line_width / 2;

            // perform a small offset we don't enlarge small features of the support
            Polygons horizontal_expansion = layer_this;
            for (coord_t offset_cumulative = 0; offset_cumulative <= extension_offset; offset_cumulative += offset_per_step)
            {
                horizontal_expansion = horizontal_expansion.offset(offset_per_step);
                model_outline = model_outline.difference(horizontal_expansion);
                model_outline = model_outline.offset(offset_per_step);
                horizontal_expansion = horizontal_expansion.difference(model_outline);
            }
            layer_this = layer_this.unionPolygons(horizontal_expansion);
        }

        if (use_towers && ! is_support_mesh_place_holder)
        {
            // handle straight walls
            AreaSupport::handleWallStruts(infill_settings, layer_this);
            // handle towers
            AreaSupport::handleTowers(infill_settings, xy_disallowed_per_layer[layer_idx], layer_this, tower_roofs, mesh.overhang_points, layer_idx, layer_count);
        }

        if (layer_idx + 1 < layer_count)
        { // join with support from layer up
            const Polygons empty;
            const Polygons* layer_above = (layer_idx < support_areas.size()) ? &support_areas[layer_idx + 1] : &empty;
            const Polygons model_mesh_on_layer
                = (layer_idx > 0) && ! is_support_mesh_nondrop_place_holder ? storage.getLayerOutlines(layer_idx, no_support, no_prime_tower) : empty;
            if (is_support_mesh_nondrop_place_holder)
            {
                layer_above = &empty;
                layer_this = layer_this.unionPolygons(storage.support.supportLayers[layer_idx].support_mesh);
            }
            layer_this = AreaSupport::join(storage, *layer_above, layer_this, smoothing_distance).difference(model_mesh_on_layer);
        }

        // make towers for small support
        if (use_towers)
        {
            for (PolygonsPart poly : layer_this.splitIntoParts())
            {
                const auto polygon_part = poly.difference(xy_disallowed_per_layer[layer_idx]).offset(-half_min_feature_width).offset(half_min_feature_width);

                const int64_t part_area = polygon_part.area();
                if (part_area == 0 || part_area > max_tower_supported_diameter * max_tower_supported_diameter)
                {
                    continue;
                }

                constexpr size_t tower_top_layer_count = 6; // number of layers after which to conclude that a tiny support area needs a tower
                if (layer_idx < layer_count - tower_top_layer_count && layer_idx >= tower_top_layer_count + bottom_empty_layer_count)
                {
                    Polygons tiny_tower_here;
                    tiny_tower_here.add(polygon_part);
                    tower_roofs.emplace_back(tiny_tower_here);
                }
            }
        }

        if (is_support_mesh_drop_down_place_holder && storage.support.supportLayers[layer_idx].support_mesh_drop_down.size() > 0)
        { // handle support mesh which should be supported by more support
            layer_this = layer_this.unionPolygons(storage.support.supportLayers[layer_idx].support_mesh_drop_down);
        }

        // Move up from model, handle stair-stepping.
        moveUpFromModel(
            storage,
            stair_removal,
            sloped_areas_per_layer[layer_idx],
            layer_this,
            layer_idx,
            bottom_empty_layer_count,
            bottom_stair_step_layer_count,
            bottom_stair_step_width);

        support_areas[layer_idx] = layer_this;
        Progress::messageProgress(Progress::Stage::SUPPORT, layer_count * (mesh_idx + 1) - layer_idx, layer_count * storage.meshes.size());
    }

    // Removal of the x/y distance needs to be outside the main loop
    // doing this in the main loop would result in more smooth support
    // structure. However, it would also remove polygon-parts close to the
    // model. For surfaces close to the maximum overhang angle no support
    // would be generated at all.
    for (auto [support_layer, xy_disallowed_area] : ranges::views::zip(support_areas, xy_disallowed_per_layer))
    {
        // inset using X/Y distance
        if (! support_layer.empty() && ! xy_disallowed_area.empty())
        {
            support_layer = support_layer.difference(xy_disallowed_area);
        }

        // Perform close operation to remove areas from support area that are unprintable
        support_layer = support_layer.offset(-half_min_feature_width).offset(half_min_feature_width);

        // remove areas smaller than the minimum support area
        support_layer.removeSmallAreas(minimum_support_area);
    }

    // do stuff for when support on buildplate only
    if (support_type == ESupportType::PLATFORM_ONLY)
    {
        Polygons touching_buildplate = support_areas[0]; // TODO: not working for conical support!
        const AngleRadians conical_support_angle = infill_settings.get<AngleRadians>("support_conical_angle");
        coord_t conical_support_offset;
        if (conical_support_angle > 0)
        { // outward ==> wider base than overhang
            conical_support_offset = -(tan(conical_support_angle) - 0.01) * layer_thickness;
        }
        else
        { // inward ==> smaller base than overhang
            conical_support_offset = (tan(-conical_support_angle) - 0.01) * layer_thickness;
        }
        const bool conical_support = infill_settings.get<bool>("support_conical_enabled") && conical_support_angle != 0;
        for (LayerIndex layer_idx = 1; layer_idx < storage.support.supportLayers.size(); layer_idx++)
        {
            const Polygons& layer = support_areas[layer_idx];

            if (conical_support)
            { // with conical support the next layer is allowed to be larger than the previous
                touching_buildplate = touching_buildplate.offset(std::abs(conical_support_offset) + 10, ClipperLib::jtMiter, 10);
                // + 10 and larger miter limit cause performing an outward offset after an inward offset can disregard sharp corners
                //
                // conical support can make
                //  layer above    layer below
                //    v              v
                //  |               : |
                //  |        ==>    : |__
                //  |____           :....
                //
                // a miter limit would result in
                //  | :             : |
                //  | :..    <==    : |__
                //  .\___           :....
                //
            }

            touching_buildplate = layer.intersection(touching_buildplate); // from bottom to top, support areas can only decrease!

            support_areas[layer_idx] = touching_buildplate;
        }
    }

    // Enforce top Z distance.
    if (layer_z_distance_top > 1)
    {
        // this is performed after the main support generation loop above, because it affects the joining of polygons
        // if this would be performed in the main loop then some support would not have been generated under the overhangs and consequently no support is generated for that,
        // meaning almost no support would be generated in some cases which definitely need support.
        const int max_checking_layer_idx
            = std::max(0, std::min(static_cast<int>(storage.support.supportLayers.size()), static_cast<int>(layer_count - (layer_z_distance_top - 1))));

        cura::parallel_for<size_t>(
            0,
            max_checking_layer_idx,
            [&](const size_t layer_idx)
            {
                constexpr bool no_support = false;
                constexpr bool no_prime_tower = false;
                support_areas[layer_idx] = support_areas[layer_idx].difference(storage.getLayerOutlines(layer_idx + layer_z_distance_top - 1, no_support, no_prime_tower));
            });
    }

    // Procedure to remove floating support
    for (size_t layer_idx = 1; layer_idx < layer_count - 1; layer_idx++)
    {
        Polygons& layer_this = support_areas[layer_idx];

        if (! layer_this.empty())
        {
            Polygons& layer_below = support_areas[layer_idx - 1];
            Polygons& layer_above = support_areas[layer_idx + 1];
            Polygons surrounding_layer = layer_above.unionPolygons(layer_below);
            layer_this = layer_this.intersection(surrounding_layer);
        }
    }

    for (size_t layer_idx = support_areas.size() - 1; layer_idx != static_cast<size_t>(std::max(-1, storage.support.layer_nr_max_filled_layer)); layer_idx--)
    {
        if (support_areas[layer_idx].size() > 0)
        {
            storage.support.layer_nr_max_filled_layer = layer_idx;
            break;
        }
    }

    storage.support.generated = true;
}

void AreaSupport::moveUpFromModel(
    const SliceDataStorage& storage,
    Polygons& stair_removal,
    Polygons& sloped_areas,
    Polygons& support_areas,
    const size_t layer_idx,
    const size_t bottom_empty_layer_count,
    const size_t bottom_stair_step_layer_count,
    const coord_t support_bottom_stair_step_width)
{
    // The idea behind support bottom stairs:
    //
    //   LEGEND:
    //   A: support resting on model
    //   x: stair step width
    //   C: to be removed from support until the next stair step = intersection between model below and A offset by x
    //
    //     ALGORITHM                                                      RESULT
    //
    // ###########################                                     ###########################                              .
    //    support                                                         support                                               .
    // ###########################   ┐                                 ###########################                              .
    // AAAAxxxxxxxxxxxxxx            │                                                                                          .
    // CCCCCCCCCCCC                  │                                 ____        ###############                              .
    // |   \      :                  │                                 |   \                                                    .
    // |____\     :                  ├> stair step height              |____\      ###############                              .
    // |     \    :                  │                                 |     \                                                  .
    // |______\   :                  │                                 |______\    ###############                              .
    // |       \  :                  │                                 |       \                                                .
    // |________\ :                  │                                 |________\  ###############                              .
    // |model    \:                  │                                 |model    \                                              .
    // |__________\###############   ┘                                 |__________\###############                              .
    // |           \                                                   |           \                                            .
    // |____________\                                                  |____________\                                           .
    //
    //
    //
    //
    //       for more horizontal surface, the stepping is (partly) negated
    //
    // ############################################################################
    //                                                    support
    // ############################################################################     ┐
    // AAAAAxxxxxxxxxxxxx                                                               │
    // CCCCCCCCCCCCCCCCCC##########################################################     │           >>>>>>>>>   result only applies stair step to first layer(s) of what woud
    // normally be the stair step
    //      ^^--..__                                                                    │
    //              ^^--..__#######################################################     ├> stair step height
    // ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺  ^^--..__                                                    │
    //                              ^^--..__#######################################     │
    // ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺ ^^--..__                                    │
    //                                              ^^--..__#######################     │
    // ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺^^--..__                    │
    //                                                              ^^--..__#######     ┘
    // ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺
    if (layer_idx < bottom_empty_layer_count)
    {
        return;
    }
    if (bottom_empty_layer_count <= 0 && bottom_stair_step_layer_count <= 1)
    {
        return;
    }

    const size_t bottom_layer_nr = layer_idx - bottom_empty_layer_count;
    constexpr bool no_support = false;
    constexpr bool no_prime_tower = false;
    const Polygons bottom_outline = storage.getLayerOutlines(bottom_layer_nr, no_support, no_prime_tower);

    Polygons to_be_removed;
    if (bottom_stair_step_layer_count <= 1)
    {
        to_be_removed = bottom_outline;
    }
    else
    {
        to_be_removed = stair_removal.unionPolygons(bottom_outline);
        if (layer_idx % bottom_stair_step_layer_count == 0)
        { // update stairs for next step
            const Polygons supporting_bottom = storage.getLayerOutlines(bottom_layer_nr - 1, no_support, no_prime_tower);
            const Polygons allowed_step_width = supporting_bottom.offset(support_bottom_stair_step_width).intersection(sloped_areas);

            const int64_t step_bottom_layer_nr = bottom_layer_nr - bottom_stair_step_layer_count + 1;
            if (step_bottom_layer_nr >= 0)
            {
                const Polygons step_bottom_outline = storage.getLayerOutlines(step_bottom_layer_nr, no_support, no_prime_tower);
                stair_removal = step_bottom_outline.intersection(allowed_step_width);
            }
            else
            {
                stair_removal = allowed_step_width;
            }
        }
    }
    support_areas = support_areas.difference(to_be_removed);
}


/*            layer 2
 * layer 1 ______________|
 * _______|         ^^^^^ basic overhang
 *
 * ^^^^^^^ supporter
 * ^^^^^^^^^^^^^^^^^ supported
 * ^^^^^^^^^^^^^^^^^^^^^^ supportee
 *         ^^^^^^^^^^^^^^^^^^^^^^^^ overhang extended
 *         ^^^^^^^^^      overhang extensions
 *         ^^^^^^^^^^^^^^ overhang
 */
std::pair<Polygons, Polygons> AreaSupport::computeBasicAndFullOverhang(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const LayerIndex& layer_idx)
{
    const Polygons outlines = mesh.layers[layer_idx].getOutlines();
    constexpr bool no_support = false;
    constexpr bool no_prime_tower = false;

    constexpr double smooth_height = 0.4; // mm
    const LayerIndex layers_below{ static_cast<LayerIndex::value_type>(std::round(smooth_height / mesh.settings.get<double>("layer_height"))) };

    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const AngleRadians support_angle = mesh.settings.get<AngleRadians>("support_angle");
    const double tan_angle = tan(support_angle) - 0.01; // The X/Y component of the support angle. 0.01 to make 90 degrees work too.
    // overhang areas protruding less then `max_dist_from_lower_layer` don't need support
    const coord_t max_dist_from_lower_layer = tan_angle * layer_height; // Maximum horizontal distance that can be bridged.

    // To avoids generating support for textures on vertical surfaces, a moving average
    // is taken over smooth_height. The smooth_height is currently an educated guess
    // that we might want to expose to the frontend in the future.
    Polygons outlines_below = storage.getLayerOutlines(layer_idx - 1, no_support, no_prime_tower).offset(max_dist_from_lower_layer);
    for (int layer_idx_offset = 2; layer_idx - layer_idx_offset >= 0 && layer_idx_offset <= layers_below; layer_idx_offset++)
    {
        auto outlines_below_ = storage.getLayerOutlines(layer_idx - layer_idx_offset, no_support, no_prime_tower).offset(max_dist_from_lower_layer * layer_idx_offset);
        outlines_below = outlines_below.unionPolygons(outlines_below_);
    }

    Polygons basic_overhang = outlines.difference(outlines_below);

    const SupportLayer& support_layer = storage.support.supportLayers[layer_idx];
    if (! support_layer.anti_overhang.empty())
    {
        // Merge anti overhang into one polygon, otherwise overlapping polygons
        // will create opposite effect.
        Polygons merged_polygons = support_layer.anti_overhang.unionPolygons();

        basic_overhang = basic_overhang.difference(merged_polygons);
    }

    Polygons overhang_extended = basic_overhang
                                     // +0.1mm for easier joining with support from layer above
                                     .offset(max_dist_from_lower_layer * layers_below + MM2INT(0.1));
    Polygons full_overhang = overhang_extended.intersection(outlines);

    return std::make_pair(basic_overhang, full_overhang);
}


void AreaSupport::detectOverhangPoints(const SliceDataStorage& storage, SliceMeshStorage& mesh)
{
    const ExtruderTrain& infill_extruder = mesh.settings.get<ExtruderTrain&>("support_infill_extruder_nr");
    const coord_t max_tower_supported_diameter = mesh.settings.get<coord_t>("support_tower_maximum_supported_diameter");
    const coord_t max_tower_supported_area = max_tower_supported_diameter * max_tower_supported_diameter;

    mesh.overhang_points.resize(storage.print_layer_count);

    for (size_t layer_idx = 1; layer_idx < storage.print_layer_count; layer_idx++)
    {
        const SliceLayer& layer = mesh.layers[layer_idx];
        const SliceLayer& layer_below = mesh.layers[layer_idx - 1];

        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.outline.outerPolygon().area() >= max_tower_supported_area)
            {
                // area is too big for support towers, should be supported by normal overhang detection
                continue;
            }

            auto has_support_below = ! layer_below.getOutlines().intersection(part.outline).empty();
            if (has_support_below)
            {
                continue;
            }

            const Polygons overhang = part.outline.difference(storage.support.supportLayers[layer_idx].anti_overhang);
            if (! overhang.empty())
            {
                scripta::log("support_overhangs", overhang, SectionType::SUPPORT, layer_idx);
                mesh.overhang_points[layer_idx].push_back(overhang);
            }
        }
    }
}

void AreaSupport::handleTowers(
    const Settings& settings,
    const Polygons& xy_disallowed_area,
    Polygons& supportLayer_this,
    std::vector<Polygons>& tower_roofs,
    std::vector<std::vector<Polygons>>& overhang_points,
    LayerIndex layer_idx,
    size_t layer_count)
{
    LayerIndex layer_overhang_point = layer_idx + 1; // Start tower 1 layer below overhang point.
    if (layer_overhang_point >= static_cast<LayerIndex>(layer_count) - 1)
    {
        return;
    }
    std::vector<Polygons>& overhang_points_here = overhang_points[layer_overhang_point]; // may be changed if an overhang point has a (smaller) overhang point directly below
    // handle new tower rooftops
    if (overhang_points_here.size() > 0)
    {
        // make sure we have the lowest point (make polys empty if they have small parts below)
        if (layer_overhang_point < static_cast<LayerIndex>(layer_count) && ! overhang_points[layer_overhang_point - 1].empty())
        {
            const auto max_tower_supported_diameter = settings.get<coord_t>("support_tower_maximum_supported_diameter");
            std::vector<Polygons>& overhang_points_below = overhang_points[layer_overhang_point - 1];
            for (Polygons& poly_here : overhang_points_here)
            {
                for (const Polygons& poly_below : overhang_points_below)
                {
                    poly_here = poly_here.difference(poly_below.offset(max_tower_supported_diameter * 2));
                }
            }
        }
        for (Polygons& poly : overhang_points_here)
        {
            if (poly.size() > 0)
            {
                tower_roofs.push_back(poly);
            }
        }
    }

    // make tower roofs
    const coord_t layer_thickness = settings.get<coord_t>("layer_height");
    const AngleRadians tower_roof_angle = settings.get<AngleRadians>("support_tower_roof_angle");
    const coord_t tower_diameter = settings.get<coord_t>("support_tower_diameter");
    const auto support_line_width = settings.get<coord_t>("support_line_width");

    coord_t tower_roof_expansion_distance;
    if (tower_roof_angle == 0)
    {
        // maximum expansion distance needed to reach a tower_diameter^2 tower area
        tower_roof_expansion_distance = tower_diameter / 2;
    }
    else
    {
        const double tan_tower_roof_angle = tan(tower_roof_angle);
        tower_roof_expansion_distance = layer_thickness / tan_tower_roof_angle;
    }

    for (Polygons& tower_roof : tower_roofs
                                    | ranges::views::filter(
                                        [](const auto& poly)
                                        {
                                            return ! poly.empty();
                                        }))
    {
        supportLayer_this = supportLayer_this.unionPolygons(tower_roof);

        if (tower_roof.area() < tower_diameter * tower_diameter)
        {
            constexpr bool no_support = false;
            constexpr bool no_prime_tower = false;
            Polygons model_outline = xy_disallowed_area;

            // Rather than offsetting the tower with tower_roof_expansion_distance we do this step wise to achieve two things
            // - prevent support from folding around the model
            // - provide method to early out the offsetting procedure when the desired area is reached
            const coord_t offset_per_step = std::max(support_line_width / 2, tower_roof_expansion_distance);
            for (coord_t offset_cumulative = 0; offset_cumulative <= tower_roof_expansion_distance; offset_cumulative += offset_per_step)
            {
                tower_roof = tower_roof.offset(offset_per_step, ClipperLib::jtRound);
                model_outline = model_outline.difference(tower_roof);
                model_outline = model_outline.offset(offset_per_step, ClipperLib::jtRound);
                tower_roof = tower_roof.difference(model_outline);

                if (tower_roof.empty())
                {
                    // guard against the tower_roof getting empty; without this guard an empty tower
                    // could indefinitely stay within the towers list
                    break;
                }
                if (tower_roof.area() >= tower_diameter * tower_diameter)
                {
                    // the desired size of the roof tower is reached, add the support tower to the
                    // current layer and clear the support tower itself
                    supportLayer_this = supportLayer_this.unionPolygons(tower_roof);
                    tower_roof.clear();
                    break;
                }
            }
        }
        else
        {
            tower_roof.clear();
        }
    }
}

void AreaSupport::handleWallStruts(const Settings& settings, Polygons& supportLayer_this)
{
    const coord_t max_tower_supported_diameter = settings.get<coord_t>("support_tower_maximum_supported_diameter");
    const coord_t tower_diameter = settings.get<coord_t>("support_tower_diameter");
    for (unsigned int p = 0; p < supportLayer_this.size(); p++)
    {
        PolygonRef poly = supportLayer_this[p];
        if (poly.size() < 6) // might be a single wall
        {
            PolygonRef poly = supportLayer_this[p];
            int best = -1;
            int best_length2 = -1;
            for (unsigned int i = 0; i < poly.size(); i++)
            {
                int length2 = vSize2(poly[i] - poly[(i + 1) % poly.size()]);
                if (length2 > best_length2)
                {
                    best = i;
                    best_length2 = length2;
                }
            }

            if (best_length2 < max_tower_supported_diameter * max_tower_supported_diameter)
            {
                break; // this is a small area, not a wall!
            }

            // an estimate of the width of the area
            int width = sqrt(poly.area() * poly.area() / best_length2); // sqrt (a^2 / l^2) instead of a / sqrt(l^2)

            // add square tower (strut) in the middle of the wall
            if (width < max_tower_supported_diameter)
            {
                Point mid = (poly[best] + poly[(best + 1) % poly.size()]) / 2;
                Polygons struts;
                PolygonRef strut = struts.newPoly();
                strut.add(mid + Point(tower_diameter / 2, tower_diameter / 2));
                strut.add(mid + Point(-tower_diameter / 2, tower_diameter / 2));
                strut.add(mid + Point(-tower_diameter / 2, -tower_diameter / 2));
                strut.add(mid + Point(tower_diameter / 2, -tower_diameter / 2));
                supportLayer_this = supportLayer_this.unionPolygons(struts);
            }
        }
    }
}

void AreaSupport::generateSupportBottom(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const size_t bottom_layer_count = round_divide(mesh.settings.get<coord_t>("support_bottom_height"), layer_height); // Number of layers in support bottom.
    if (bottom_layer_count <= 0)
    {
        return;
    }
    const coord_t z_distance_bottom = round_up_divide(mesh.settings.get<coord_t>("support_bottom_distance"), layer_height); // Number of layers between support bottom and model.
    const size_t skip_layer_count
        = std::max(uint64_t(1), round_divide(mesh.settings.get<coord_t>("support_interface_skip_height"), layer_height)); // Resolution of generating support bottoms above model.
    const coord_t bottom_line_width = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").settings.get<coord_t>("support_bottom_line_width");
    const coord_t bottom_outline_offset = mesh_group_settings.get<ExtruderTrain&>("support_bottom_extruder_nr").settings.get<coord_t>("support_bottom_offset");

    const size_t scan_count = std::max(size_t(1), (bottom_layer_count - 1) / skip_layer_count); // How many measurements to take to generate bottom areas.
    const float z_skip = std::max(
        1.0f,
        float(bottom_layer_count - 1) / float(scan_count)); // How many layers to skip between measurements. Using float for better spread, but this is later rounded.
    const double minimum_bottom_area = mesh.settings.get<double>("minimum_bottom_area");

    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (LayerIndex layer_idx = support_layers.size() - 1; layer_idx >= static_cast<int>(z_distance_bottom); --layer_idx)
    {
        const unsigned int bottom_layer_idx_below = std::max(0, int(layer_idx) - int(bottom_layer_count) - int(z_distance_bottom));
        Polygons mesh_outlines;
        for (float layer_idx_below = bottom_layer_idx_below; std::round(layer_idx_below) < (int)(layer_idx - z_distance_bottom); layer_idx_below += z_skip)
        {
            mesh_outlines.add(mesh.layers[std::round(layer_idx_below)].getOutlines());
        }
        Polygons bottoms;
        generateSupportInterfaceLayer(global_support_areas_per_layer[layer_idx], mesh_outlines, bottom_line_width, bottom_outline_offset, minimum_bottom_area, bottoms);
        support_layers[layer_idx].support_bottom.add(bottoms);
        scripta::log("support_interface_bottoms", bottoms, SectionType::SUPPORT, layer_idx);
    }
}

void AreaSupport::generateSupportRoof(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const size_t roof_layer_count = round_divide(mesh.settings.get<coord_t>("support_roof_height"), layer_height); // Number of layers in support roof.
    if (roof_layer_count <= 0)
    {
        return;
    }
    const coord_t z_distance_top = round_up_divide(mesh.settings.get<coord_t>("support_top_distance"), layer_height); // Number of layers between support roof and model.
    const size_t skip_layer_count
        = std::max(uint64_t(1), round_divide(mesh.settings.get<coord_t>("support_interface_skip_height"), layer_height)); // Resolution of generating support roof below model.
    const coord_t roof_line_width = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").settings.get<coord_t>("support_roof_line_width");
    const coord_t roof_outline_offset = mesh_group_settings.get<ExtruderTrain&>("support_roof_extruder_nr").settings.get<coord_t>("support_roof_offset");

    const size_t scan_count = std::max(size_t(1), (roof_layer_count - 1) / skip_layer_count); // How many measurements to take to generate roof areas.
    const float z_skip = std::max(
        1.0f,
        float(roof_layer_count - 1) / float(scan_count)); // How many layers to skip between measurements. Using float for better spread, but this is later rounded.
    const double minimum_roof_area = mesh.settings.get<double>("minimum_roof_area");

    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (LayerIndex layer_idx = static_cast<int>(support_layers.size() - z_distance_top) - 1; layer_idx >= 0; --layer_idx)
    {
        const LayerIndex top_layer_idx_above{
            std::min(LayerIndex{ support_layers.size() - 1 }, LayerIndex{ layer_idx + roof_layer_count + z_distance_top })
        }; // Maximum layer of the model that generates support roof.
        Polygons mesh_outlines;
        for (float layer_idx_above = top_layer_idx_above; layer_idx_above > layer_idx + z_distance_top; layer_idx_above -= z_skip)
        {
            mesh_outlines.add(mesh.layers[std::round(layer_idx_above)].getOutlines());
        }
        Polygons roofs;
        generateSupportInterfaceLayer(global_support_areas_per_layer[layer_idx], mesh_outlines, roof_line_width, roof_outline_offset, minimum_roof_area, roofs);
        support_layers[layer_idx].support_roof.add(roofs);
        if (layer_idx > 0 && layer_idx < support_layers.size() - 1)
        {
            support_layers[layer_idx].support_fractional_roof.add(roofs.difference(support_layers[layer_idx + 1].support_roof));
        }
        scripta::log("support_interface_roofs", roofs, SectionType::SUPPORT, layer_idx);
    }

    // Remove support in between the support roof and the model. Subtracts the roof polygons from the support polygons on the layers above it.
    for (auto [layer_idx, support_layer] : support_layers | ranges::views::enumerate | ranges::views::drop(1) | ranges::views::drop_last(z_distance_top))
    {
        if (support_layer.support_roof.empty())
        {
            continue;
        }

        int lower = static_cast<int>(layer_idx);
        int upper = std::min(static_cast<int>(layer_idx + roof_layer_count + z_distance_top + 5), static_cast<int>(global_support_areas_per_layer.size()) - 1);
        for (Polygons& global_support : global_support_areas_per_layer | ranges::views::slice(lower, upper))
        {
            global_support = global_support.difference(support_layer.support_roof);
        }
    }
}

void AreaSupport::generateSupportInterfaceLayer(
    Polygons& support_areas,
    const Polygons colliding_mesh_outlines,
    const coord_t safety_offset,
    const coord_t outline_offset,
    const double minimum_interface_area,
    Polygons& interface_polygons)
{
    Polygons model = colliding_mesh_outlines.unionPolygons();
    interface_polygons = support_areas.offset(safety_offset / 2).intersection(model);
    interface_polygons = interface_polygons.offset(safety_offset).intersection(support_areas); // Make sure we don't generate any models that are not printable.
    if (outline_offset != 0)
    {
        interface_polygons = interface_polygons.offset(outline_offset);
        if (outline_offset > 0) // The interface might exceed the area of the normal support.
        {
            interface_polygons = interface_polygons.intersection(support_areas);
        }
    }
    if (minimum_interface_area > 0.0)
    {
        interface_polygons.removeSmallAreas(minimum_interface_area);
    }
    support_areas = support_areas.difference(interface_polygons);
}

} // namespace cura
