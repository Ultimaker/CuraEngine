#ifndef CURAENGINE_TREESUPPORTCRADLE_H
#define CURAENGINE_TREESUPPORTCRADLE_H
#include <spdlog/spdlog.h>

#include "TreeModelVolumes.h"
#include "TreeSupportEnums.h"
#include "TreeSupportSettings.h"
#include "polyclipping/clipper.hpp"
#include "settings/types/LayerIndex.h"
#include "sliceDataStorage.h"
#include "utils/Coord_t.h"
#include "utils/MinimumBoundingBox.h"

namespace cura
{
// todo rename file as now general TreeSupportTipDataStructures
struct TreeSupportCradle;

using CradleDeformationHalfCircle = std::array<double, 16>;

struct OverhangInformation
{
    OverhangInformation(Shape overhang, bool roof)
        : overhang_(overhang)
        , is_roof_(roof)
        , is_cradle_(false)
        , cradle_layer_idx_(-1)
        , cradle_line_idx_(-1)
        , cradle_(nullptr)
    {
    }

    OverhangInformation(Shape overhang, bool roof, TreeSupportCradle* cradle, int32_t cradle_layer_idx = -1, int32_t cradle_line_idx = -1)
        : overhang_(overhang)
        , is_roof_(roof)
        , is_cradle_(true)
        , cradle_layer_idx_(cradle_layer_idx)
        , cradle_line_idx_(cradle_line_idx)
        , cradle_(cradle)
    {
    }

    Shape overhang_;
    bool is_roof_;
    bool is_cradle_;
    int32_t cradle_layer_idx_;
    int32_t cradle_line_idx_;
    TreeSupportCradle* cradle_;

    bool isCradleLine()
    {
        return is_cradle_ && cradle_line_idx_ >= 0 && cradle_layer_idx_ >= 0;
    }
};
struct TreeSupportCradleLine
{
    // required to shrink a vector using resize
    TreeSupportCradleLine()
    {
        spdlog::error("Dummy TreeSupportCradleLine constructor called");
    }

    TreeSupportCradleLine(OpenPolyline line, LayerIndex layer_idx, bool is_roof)
        : line_(line)
        , layer_idx_(layer_idx)
        , is_roof_(is_roof)
    {
    }
    Shape area_;
    OpenPolyline line_;
    OpenPolyline removed_line_;
    LayerIndex layer_idx_;
    bool is_base_ = false;
    bool is_roof_;

    void addLineToRemoved(OpenPolyline& line_to_add)
    {
        if (removed_line_.empty())
        {
            removed_line_.push_back(line_to_add.front());
            removed_line_.push_back(line_to_add.back());
        }
        else
        {
            if (vSize2(removed_line_.front() - removed_line_.back()) < vSize2(line_to_add.front() - removed_line_.back()))
            {
                removed_line_.front() = line_to_add.front();
            }

            if (vSize2(removed_line_.front() - removed_line_.back()) < vSize2(removed_line_.front() - line_to_add.back()))
            {
                removed_line_.back() = line_to_add.back();
            }
        }
    }
};

struct CradleConfig
{
    CradleConfig(const SliceMeshStorage& mesh, bool roof):
        cradle_layers_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_height") / mesh.settings.get<coord_t>("layer_height"))
        , cradle_layers_min_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_min_height") / mesh.settings.get<coord_t>("layer_height"))
        , cradle_line_count_(retrieveSetting<size_t>(mesh.settings, "support_tree_cradle_line_count"))
        , cradle_length_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_length"))
        , cradle_length_min_(retrieveSetting<coord_t>(mesh.settings, "support_tree_min_cradle_length"))
        , cradle_line_width_(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_line_width"))
        , cradle_lines_roof_(roof && retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") != "none")
        , cradle_base_roof_(roof && (
                                      retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") == "cradle_and_base" ||
                                      retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") == "large_cradle_and_base"
                                    ))
        , large_cradle_base_(roof && retrieveSetting<std::string>(mesh.settings, "support_tree_roof_cradle") == "large_cradle_and_base")
        , large_cradle_line_tips_(retrieveSetting<bool>(mesh.settings, "support_tree_large_cradle_line_tips"))
        , cradle_z_distance_layers_(round_divide(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_z_distance"), mesh.settings.get<coord_t>("layer_height")))
        , cradle_towards_center_(retrieveSetting<std::string>(mesh.settings, "support_tree_cradle_direction")  == "center")
    {
        TreeSupportSettings config(mesh.settings); //todo replace with gathering settings manually
        if (cradle_layers_)
        {
            cradle_layers_ += cradle_z_distance_layers_;
        }
        if (cradle_length_ - cradle_line_width_ > 0)
        {
            cradle_length_ -= cradle_line_width_;
            cradle_length_min_ -= cradle_line_width_;
        }

        cradle_z_distance_layers_ = round_divide(retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_z_distance"), config.layer_height);
        cradle_support_base_area_radius_ = retrieveSetting<coord_t>(mesh.settings, "support_tree_cradle_base_diameter") / 2;

        for (size_t cradle_z_dist_ctr = 0; cradle_z_dist_ctr < cradle_z_distance_layers_ + 1; cradle_z_dist_ctr++)
        {
            cradle_xy_distance_.emplace_back(config.xy_min_distance);
        }

        for (int i = 0; i < 9; i++)
        {
            std::stringstream setting_name_dist;
            setting_name_dist << "support_tree_cradle_xy_dist_";
            setting_name_dist << i;
            coord_t next_cradle_xy_dist = retrieveSetting<coord_t>(mesh.settings, setting_name_dist.str());
            std::stringstream setting_name_height;
            setting_name_height << "support_tree_cradle_xy_height_";
            setting_name_height << i;
            coord_t next_cradle_xy_dist_height = retrieveSetting<coord_t>(mesh.settings, setting_name_height.str());
            if (next_cradle_xy_dist_height == 0)
            {
                break;
            }

            for (int layer_delta = 0; layer_delta < round_up_divide(next_cradle_xy_dist_height, config.layer_height); layer_delta++)
            {
                cradle_xy_distance_.emplace_back(next_cradle_xy_dist);
            }
        }

        for (int cradle_xy_dist_fill = cradle_xy_distance_.size(); cradle_xy_dist_fill <= cradle_layers_ + 1; cradle_xy_dist_fill++)
        {
            cradle_xy_distance_.emplace_back(config.xy_min_distance);
        }

        min_distance_between_lines_areas_ = (config.fill_outline_gaps ? config.min_feature_size / 2 - 5 : config.min_wall_line_width / 2 - 5);
        cradle_line_distance_ = min_distance_between_lines_areas_ + config.xy_distance;
    }

    /*!
     * \brief Amount of layers of the cradle to support pointy overhangs.
     */
    size_t cradle_layers_;

    /*!
     * \brief Minimum amount of layers of the cradle to support pointy overhangs.
     */
    size_t cradle_layers_min_;

    /*!
     * \brief Amount of lines used for the cradle.
     */
    size_t cradle_line_count_;

    /*!
     * \brief Length of lines used for the cradle.
     */
    coord_t cradle_length_;

    /*!
     * \brief Minimum length of lines used for the cradle. TODO Width is effectively added to length ... fix or document?
     */
    coord_t cradle_length_min_;

    /*!
     * \brief Width of lines used for the cradle.
     */
    coord_t cradle_line_width_;

    /*!
     * \brief If cradle lines should be drawn as roof.
     */
    bool cradle_lines_roof_;

    /*!
     * \brief If the cradle base should be drawn as roof.
     */
    bool cradle_base_roof_;

    /*!
     * \brief Distances the cradle lines should be from the model. First value corresponds to cradle line on the same layer as the first model line.
     */
    std::vector<coord_t> cradle_xy_distance_;

    /*!
     * \brief If the (roof) cradle base should contain all cradle lines at cradle_tip_dtt size.
     */
    bool large_cradle_base_;

    /*!
     * \brief If the cradle lines should also be supported by larger tips.
     */
    bool large_cradle_line_tips_;

    /*!
     * \brief Distances in lines between the cradle and the support they are supported by.
     */
    size_t cradle_z_distance_layers_;

    /*!
     * \brief Distance to top of tips that support either the pointy overhang or the cradle lines at the bottom-most layer.
     */
    coord_t cradle_support_base_area_radius_;

    /*!
     * \brief Distance between line areas to prevent them from fusing
     */
    coord_t min_distance_between_lines_areas_;

    /*!
     * \brief Targeted distance between line areas
     */
    coord_t cradle_line_distance_;

    /*!
     * \brief Should cradle lines go toward the center of the model (true) or the closest point on the outline(false)
     */
    bool cradle_towards_center_;

};

struct TreeSupportCradle
{
    std::vector<std::deque<TreeSupportCradleLine>> lines_;
    bool is_roof_;
    LayerIndex layer_idx_;
    std::vector<Shape> base_below_;
    std::vector<Point2LL> centers_;
    std::vector<Shape> shadow_;
    std::vector<Shape> part_outline_;
    std::unordered_map<LayerIndex, std::vector<OverhangInformation>> overhang_;
    CradlePlacementMethod cradle_placement_method_;
    const std::shared_ptr<const CradleConfig> config_;
    size_t mesh_idx_;


    TreeSupportCradle(LayerIndex layer_idx, Point2LL center, bool roof, CradlePlacementMethod cradle_placement_method, std::shared_ptr<const CradleConfig> config, size_t mesh_idx)
        : layer_idx_(layer_idx)
        , centers_({ center })
        , is_roof_(roof)
        , cradle_placement_method_(cradle_placement_method)
        , config_ (config)
        , mesh_idx_(mesh_idx)
    {
    }


    std::optional<TreeSupportCradleLine*> getCradleLineOfIndex(LayerIndex requested_layer_idx, size_t cradle_line_idx)
    {
        if (cradle_line_idx < lines_.size())
        {
            for (size_t height_idx = 0; height_idx < lines_[cradle_line_idx].size(); height_idx++)
            {
                if (lines_[cradle_line_idx][height_idx].layer_idx_ == requested_layer_idx)
                {
                    return &lines_[cradle_line_idx][height_idx];
                }
            }
        }
        return {};
    }

    Point2LL getCenter(LayerIndex layer_idx_req)
    {
        if (layer_idx_req < layer_idx_)
        {
            return centers_.front();
        }
        size_t index = layer_idx_req - layer_idx_;
        if (centers_.size() <= index)
        {
            return centers_.back();
        }
        return centers_[index];
    }

    size_t getIndexForLineEnd(Point2LL line_end, LayerIndex layer_idx_req)
    {
        Point2LL current_direction = line_end - getCenter(layer_idx_req);
        double angle = std::atan2(current_direction.Y, current_direction.X);
        if(angle < 0)
        {
            angle = 2.0 * std::numbers::pi + angle;
        }
        double step_size = (2.0 * std::numbers::pi) / double(config_->cradle_line_count_);
        size_t angle_idx = size_t(std::round(angle / step_size)) % config_->cradle_line_count_;
        return angle_idx;
    }

    void verifyLines()
    {
        for (size_t line_idx = 0; line_idx < lines_.size(); line_idx++)
        {
            if (lines_[line_idx].size() < config_->cradle_layers_min_)
            {
                lines_[line_idx].clear();
                continue;
            }
            LayerIndex previous_layer_idx;

            for (size_t up_idx = 0; up_idx < lines_[line_idx].size(); up_idx++)
            {
                if (! lines_[line_idx][up_idx].is_base_)
                {
                    previous_layer_idx = lines_[line_idx][up_idx].layer_idx_;
                    if (lines_[line_idx][up_idx].layer_idx_ > previous_layer_idx + up_idx || lines_[line_idx][up_idx].line_.size() < 2
                        || lines_[line_idx][up_idx].line_.length() < config_->cradle_length_min_)
                    {
                        lines_[line_idx].clear();
                    }
                    break;
                }
            }
            for (size_t up_idx = 1; up_idx < lines_[line_idx].size(); up_idx++)
            {
                if (! lines_[line_idx][up_idx].is_base_)
                {
                    if (lines_[line_idx][up_idx].layer_idx_ > previous_layer_idx + up_idx || lines_[line_idx][up_idx].line_.size() < 2
                        || lines_[line_idx][up_idx].line_.length() < config_->cradle_length_min_)
                    {
                        if (up_idx <= config_->cradle_layers_min_)
                        {
                            spdlog::debug(
                                "Removing cradle line of cradle on layer {} line at {}. Invalid line was on layer {}",
                                layer_idx_,
                                line_idx,
                                lines_[line_idx][up_idx].layer_idx_);
                            lines_[line_idx].clear();
                            break;
                        }
                        else
                        {
                            spdlog::debug("Partially removing cradle line of cradle on layer {} line at {} at height {}", layer_idx_, line_idx, up_idx);
                            lines_[line_idx].resize(up_idx - 1);
                            break;
                        }
                    }
                }
            }
        }
    }
};

struct CradlePresenceInformation
{
    CradlePresenceInformation(TreeSupportCradle* cradle, LayerIndex layer_idx, size_t line_idx)
        : cradle_(cradle)
        , layer_idx_(layer_idx)
        , line_idx_(line_idx)
    {
    }
    TreeSupportCradle* cradle_;
    LayerIndex layer_idx_;
    size_t line_idx_;

    TreeSupportCradleLine* getCradleLine()
    {
        return cradle_->getCradleLineOfIndex(layer_idx_, line_idx_).value();
    }

    bool cradleLineExists()
    {
        return cradle_->getCradleLineOfIndex(layer_idx_, line_idx_).has_value();
    }
};

class SupportCradleGeneration
{
public:
    /*!
     * \brief Add meshes to generate cradles.
     * \param storage[in] The storage that the mesh is in.
     * \param mesh_idx[in] The idx of the mesh that is currently processed.
     */
    void addMeshToCradleCalculation(const SliceDataStorage& storage, size_t mesh_idx);

    /*!
     * \brief Generates mesh-specific cradle data. Calls addMeshToCradleCalculation if mesh was not already added. Not threadsafe!
     * \param storage[in] The storage that the mesh is in.
     * \param mesh_idx[in] The idx of the mesh that is currently processed.
     */
    void generateCradleForMesh(const SliceDataStorage& storage, size_t mesh_idx);

    /*!
     * \brief Generate all cradle areas and line areas for the previously added meshes. Not threadsafe! Should only called once.
     * \param storage[in] The storage that contains all meshes. Used to access mesh specific settings
     */

    void generate(const SliceDataStorage& storage);

    /*!
     * \brief Retrieve cradles generated for a certain mesh.
     * \param target[out] Data storage for the cradles.
     * \param support_free_areas[out] Areas where support should be removed to ensure the pointy overhang to supported.
     * \param mesh_idx[in] The idx of the mesh for which the cradles are retrieved.
     */
    void pushCradleData(std::vector<std::vector<TreeSupportCradle*>>& target, std::vector<Shape>& support_free_areas, size_t mesh_idx);

    /*!
     * \brief Give the largest layer index that contains the first layer of a cradle. Only considers meshes that were added with addMeshToCradleCalculation.
     */
    LayerIndex getTopMostCradleLayer()
    {
        return top_most_cradle_layer_;
    }

    SupportCradleGeneration(const SliceDataStorage& storage, TreeModelVolumes& volumes_);
private:

    struct UnsupportedAreaInformation
    {
        UnsupportedAreaInformation(const Shape area, LayerIndex layer_idx, size_t height, coord_t accumulated_supportable_overhang,
            CradleDeformationHalfCircle deformation, MinimumBoundingBox min_box)
            : area{ area }
            , layer_idx{ layer_idx }
            , height{ height }
            , accumulated_supportable_overhang{ accumulated_supportable_overhang }
            , deformation{ deformation }
            , min_box {min_box}
        {
        }
        const Shape area;
        LayerIndex layer_idx;
        size_t height;
        coord_t accumulated_supportable_overhang;
        CradlePlacementMethod support_required = CradlePlacementMethod::NONE;
        CradleDeformationHalfCircle deformation;
        double deformation_limit_total = -1;
        double deformation_limit_z = -1;

        double deformation_total_calculated = -1;
        MinimumBoundingBox min_box;

        // Contains identifiers of all cradles below
        std::vector<UnsupportedAreaInformation*> cradles_below;
        std::vector<UnsupportedAreaInformation*> areas_above;
        std::vector<UnsupportedAreaInformation*> areas_below;
    };

    size_t getDirectionIdx(Point2LL a, Point2LL b) const;

    CradleDeformationHalfCircle elementWiseSelect(CradleDeformationHalfCircle& a, CradleDeformationHalfCircle& b, std::function<double(double,double)> selector)
    {
        CradleDeformationHalfCircle result{};
        for(int i = 0; i < a.size(); i++)
        {
            result[i]=selector(a[i],b[i]);
        }
        return result;
    }

    void getLayerDeformation(const SliceMeshStorage& mesh, MinimumBoundingBox& minimum_box, double assumed_part_thickness, CradleDeformationHalfCircle& deform_part);

    std::pair<MinimumBoundingBox,CradleDeformationHalfCircle > getSimulatedConnection(const SliceMeshStorage& mesh,
                                                                                      double assumed_part_thickness,
                                                                                      std::set<UnsupportedAreaInformation*> elements);

    //todo doku
    double getTotalDeformation(size_t mesh_idx, const SliceMeshStorage& mesh, UnsupportedAreaInformation* element);

    /*!
     * \brief Provides areas that do not have a connection to the buildplate or any other non support material below it.
     * \param mesh_idx[in] The idx of the mesh.
     * \param layer_idx The layer said area is on.
     * \return A vector containing the areas, how many layers of material they have below them (always 0) and the idx of each area usable to get the next one layer above.
 */
    std::vector<UnsupportedAreaInformation*> getFullyUnsupportedArea(size_t mesh_idx, LayerIndex layer_idx);

    /*!
     * \brief Calculates which parts of the model to not connect with the buildplate and how many layers of material is below them (height).
     * Results are stored in a cache.
     * Only area up to the required maximum height are stored.
     * \param storage[in] The storage meshes are in.
     * \param mesh_idx[in] The idx of the mesh.
     */
    void calculateFloatingParts(const SliceDataStorage& storage, size_t mesh_idx);

/*!
     * \brief Generate the center points of all generated cradles.
     * \param mesh[in] The mesh that is currently processed.
     * \param mesh_idx[in] The idx of the mesh.
     * \returns The calculated cradle centers for the mesh.
 */
std::vector<std::vector<TreeSupportCradle*>> generateCradleCenters(const SliceMeshStorage& mesh, size_t mesh_idx);

/*!
     * \brief Generate lines from center and model information
     * Only area up to the required maximum height are stored.
     * \param cradle_data_mesh[in] The calculated cradle centers for the mesh.
     * \param mesh[in] The mesh that is currently processed.
 */
void generateCradleLines(std::vector<std::vector<TreeSupportCradle*>>& cradle_data_mesh, const SliceMeshStorage& mesh);

/*!
     * \brief Ensures cradle-lines do not intersect with each other.
 */
void cleanCradleLineOverlaps();

/*!
     * \brief Finishes the cradle areas that represent cradle base and lines and calculates overhang for them.
     * \param storage[in] The storage that contains all meshes. Used to access mesh specific settings
 */
void generateCradleLineAreasAndBase(const SliceDataStorage& storage);



/*!
     * \brief Representation of all cradles ordered by mesh_idx and layer_idx.
 */
std::vector<std::vector<std::vector<TreeSupportCradle*>>> cradle_data_;

/*!
     * \brief Representation of areas that have to be removed to ensure lines below the pointy overhang.
 */
std::vector<Shape> support_free_areas_;

/*!
     * \brief Generator for model collision, avoidance and internal guide volumes.
 */
TreeModelVolumes& volumes_;

/*!
     * \brief Whether only support that can rest on a flat surface should be supported. todo
 */
const bool only_gracious_ = false;

LayerIndex top_most_cradle_layer_ = -1;

mutable std::vector<std::vector<std::vector<UnsupportedAreaInformation*>>> floating_parts_cache_;
mutable std::unordered_map<LayerIndex, std::vector<std::pair<std::set<UnsupportedAreaInformation*>,std::pair<MinimumBoundingBox,CradleDeformationHalfCircle>>>> simulated_connection_cache_;

std::unique_ptr<std::mutex> critical_floating_parts_cache_ = std::make_unique<std::mutex>();
std::unique_ptr<std::mutex> critical_simulated_connection_cache_ = std::make_unique<std::mutex>();



};


} // namespace cura

#endif // CURAENGINE_TREESUPPORTCRADLE_H
