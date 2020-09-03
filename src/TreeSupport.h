//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include <forward_list>
#include <unordered_set>
#include "settings/EnumSettings.h"


namespace cura
{


class ModelVolumes
{
public:

    ModelVolumes()=default;
	ModelVolumes(const SliceDataStorage& storage,const coord_t max_move,const coord_t max_move_slow,size_t current_mesh_idx,double progress_multiplier,double progress_offset,  const std::vector<Polygons>& additional_excluded_areas=std::vector<Polygons>());
    ModelVolumes(ModelVolumes&&) = default;
    ModelVolumes& operator=(ModelVolumes&&) = default;

    ModelVolumes(const ModelVolumes&) = delete;
    ModelVolumes& operator=(const ModelVolumes&) = delete;

	void precalculate(coord_t max_layer);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model.
     *
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param min_xy_dist Is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getCollision(coord_t orig_radius, LayerIndex layer_idx,bool min_xy_dist=false);

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches
     * in order to reach the build plate.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model or be unable to reach the build platform.
     *
     * The input collision areas are inset by the maximum move distance and
     * propagated upwards.
     *
     * \param orig_radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \param slow Is the propgation with the maximum move distance slow required.
     * \param to_model Does the avoidance allow good connections with the model.
     * \param min_xy_dist is the minimum xy distance used.
     * \return Polygons object
     */
    const Polygons& getAvoidance(coord_t orig_radius, LayerIndex layer_idx,bool slow=false,bool to_model=false,bool min_xy_dist=false);
    /*!
     * \brief The area represents all areas on the model where the branch does completely fit on the given layer.
     * \param radius The radius of the node of interest
     * \param layer_idx The layer of interest
     * \return Polygons object
     */
    const Polygons& getPlaceableAreas(coord_t radius, LayerIndex layer_idx);
    /*!
     * \brief Round \p radius upwards to either a multiple of radius_sample_resolution_ or a exponentially increasing value
     *
     *	It also adds the difference between the minimum xy distance and the regular one.
     *
     * \param radius The radius of the node of interest
     * \param min_xy_dist is the minimum xy distance used.
     */
    coord_t ceilRadius(coord_t radius,bool min_xy_dist) const;
private:
    /*!
     * \brief Convenience typedef for the keys to the caches
     */
    using RadiusLayerPair = std::pair<coord_t, LayerIndex>;

    /*!
     * \brief Round \p radius upwards to either a multiple of radius_sample_resolution_ or a exponentially increasing value
     *
     * \param radius The radius of the node of interest
     */
    coord_t ceilRadius(coord_t radius) const;

    Polygons extractOutlineFromMesh(const SliceMeshStorage& mesh, LayerIndex layer_idx) const;

    void calculateCollision( std::deque<RadiusLayerPair> keys );

    void calculateCollision( RadiusLayerPair key ){
    	calculateCollision(std::deque<RadiusLayerPair>{RadiusLayerPair(key)});
    }

    Polygons safeOffset(const Polygons& me,coord_t distance,ClipperLib::JoinType jt,coord_t max_safe_step_distance,const Polygons& collision)const;


    void calculateAvoidance( std::deque<RadiusLayerPair> keys );

    void calculateAvoidance( RadiusLayerPair key ){
#pragma omp parallel //required as otherwise the "omp for" inside calculateAvoidance will lock up
    	{
    	calculateAvoidance(std::deque<RadiusLayerPair>{RadiusLayerPair(key)});
    	}
    }

    void calculatePlaceables( RadiusLayerPair key ){
#pragma omp parallel
    	{
			calculatePlaceables(std::deque<RadiusLayerPair>{key});
    	}
    }


    void calculatePlaceables( std::deque<RadiusLayerPair> keys );

    void calculateAvoidanceToModel( std::deque<RadiusLayerPair> keys );

    void calculateAvoidanceToModel( RadiusLayerPair key ){
#pragma omp parallel
    	{
    		calculateAvoidanceToModel(std::deque<RadiusLayerPair>{RadiusLayerPair(key)});
    	}
    }

    const std::optional<std::reference_wrapper<const Polygons>> getArea(std::unordered_map<RadiusLayerPair, Polygons>& cache,const RadiusLayerPair key) const;
    bool checkSettingsEquality(const Settings& me, const Settings& other)const;
    coord_t getXYDistByIdx(size_t outline_idx,bool min_xy_dist) const;
    coord_t getXYDistDeltaByIdx(size_t outline_idx) const{
    	return getXYDistByIdx(outline_idx, false)-getXYDistByIdx(outline_idx, true);
    }
    LayerIndex getMaxCalculatedLayer(coord_t radius,const std::unordered_map<RadiusLayerPair, Polygons>& map ) const;
    Polygons calculateMachineBorderCollision(Polygon machine_border);
    /*!
     * \brief The maximum distance that the centrepoint of a tree branch may
     * move in consequtive layersif it has to avoid the model
     */
    coord_t max_move_;
    /*!
     * \brief The maximum distance that the centrepoint of a tree branch may
     * move in consequtive layers if it does not have to avoid the model
     */
    coord_t max_move_slow_;
    /*!
     * \brief The smallest maximum resolution for simplify
     */
	coord_t min_maximum_resolution_;
    /*!
     * \brief The smallest maximum deviation for simplify
     */
	coord_t min_maximum_deviation_;
    /*!
     * \brief Whether the precalculate was called, meaning every required value should be cached.
     */
	bool precalculated=false;
    /*!
     * \brief The index to access the outline corresponding with the currently processing mesh
     */
	size_t current_outline_idx;
    /*!
     * \brief The minimum required clearance between the model and the tree branches
     */
	coord_t current_min_xy_dist;
    /*!
     * \brief The difference between the minimum required clearance between the model and the tree branches and the regular one.
     */
	coord_t current_min_xy_dist_delta;
    /*!
     * \brief Does at least one mesh allow support to rest on a model.
     */
	bool support_rests_on_model;
    /*!
     * \brief The progress of the precalculate function for communicating it to the progress bar.
     */
	coord_t precalculation_progress=0;
    /*!
     * \brief Polygons representing the limits of the printable area of the
     * machine
     */
	double progress_multiplier;
	double progress_offset;
    Polygons machine_border_;
    /*!
     * \brief Storage for layer outlines and the corresponding settings of the meshes grouped by meshes with identical setting.
     */
    std::vector<std::pair<Settings,std::vector<Polygons>>> layer_outlines_;
    /*!
     * \brief Storage for areas that should be avoided, like support blocker or previous generated trees.
     */
    std::vector<Polygons> anti_overhang_;
    /*!
     * \brief Radiis that can be ignored by ceilRadius as they will never be requested.
     */
    std::unordered_set<coord_t> ignorable_radii_;

    /*!
     * \brief Caches for the collision, avoidance and areas on the model where support can be placed safely
     * at given radius and layer indices.
     *
     * These are mutable to allow modification from const function. This is
     * generally considered OK as the functions are still logically const
     * (ie there is no difference in behaviour for the user betweeen
     * calculating the values each time vs caching the results).
     */
    mutable std::unordered_map<RadiusLayerPair, Polygons> collision_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_slow_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_to_model_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_to_model_slow_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> placeable_areas_cache_;

};


class SliceDataStorage;
class SliceMeshStorage;
class ModelVolumes;


/*!
 * \brief Generates a tree structure to support your models.
 */

class TreeSupport
{
public:
    /*!
     * \brief Creates an instance of the tree support generator.
     *
     * \param storage The data storage to get global settings from.
     */
    TreeSupport(const SliceDataStorage& storage);

    /*!
     * \brief Create the areas that need support.
     *
     * These areas are stored inside the given SliceDataStorage object.
     * \param storage The data storage where the mesh data is gotten from and
     * where the resulting support areas are stored.
     */
    void generateSupportAreas(SliceDataStorage& storage);


    struct TreeSupportSettings; // forward declaration as we need some config values in the merge case


    struct SupportElement{

    	SupportElement(coord_t distance_to_top,size_t target_height,Point target_position,bool to_buildplate,bool to_model_gracious,bool use_min_xy_dist): //SupportElement(target_height,target_position,target_height,target_position,distance_to_top,distance_to_top,to_buildplate)
    		target_height(target_height),
			target_position(target_position),
			next_position(target_position),
			next_height(target_height),
			effective_radius_height(distance_to_top),
			to_buildplate(to_buildplate),
			distance_to_top(distance_to_top),
			area(nullptr),
			increased_ddt(0),
			to_model_gracious(to_model_gracious),
			elephant_foot_increases(0),
			use_min_xy_dist(use_min_xy_dist),
			non_gracious_model_contact(false)
    	{

    	}


    	SupportElement(const SupportElement& elem,Polygons* newArea=nullptr): // copy constructor with possibility to set a new area
    			target_height(elem.target_height),
    			target_position(elem.target_position),
    			next_position(elem.next_position),
    			next_height(elem.next_height),
    			effective_radius_height(elem.effective_radius_height),
    			to_buildplate(elem.to_buildplate),
				distance_to_top(elem.distance_to_top),
				area(newArea!=nullptr?newArea:elem.area),
				result_on_layer(elem.result_on_layer),
				increased_ddt(elem.increased_ddt),
				to_model_gracious(elem.to_model_gracious),
				elephant_foot_increases(elem.elephant_foot_increases),
				use_min_xy_dist(elem.use_min_xy_dist),
				non_gracious_model_contact(elem.non_gracious_model_contact)

    	{
    		parents.insert(parents.begin(), elem.parents.begin(), elem.parents.end());
    	}

        /*!
         * \brief Create a new Element for one layer below the element of the pointer supplied.
         */

    	SupportElement(SupportElement* element_above):
    			target_height(element_above->target_height),
    			target_position(element_above->target_position),
    			next_position(element_above->next_position),
    			next_height(element_above->next_height),
    			effective_radius_height(element_above->effective_radius_height),
    			to_buildplate(element_above->to_buildplate),
				distance_to_top(element_above->distance_to_top+1),
				area(element_above->area),
				result_on_layer(Point(-1,-1)), // set to invalid as we are a new node on a new layer
				increased_ddt(element_above->increased_ddt),
				to_model_gracious(element_above->to_model_gracious),
				elephant_foot_increases(element_above->elephant_foot_increases),
				use_min_xy_dist(element_above->use_min_xy_dist),
				non_gracious_model_contact(element_above->non_gracious_model_contact)
    	{
    		parents={element_above};
    	}

    	SupportElement(const SupportElement& first,const SupportElement& second,size_t next_height,Point next_position,coord_t increased_ddt,TreeSupportSettings config):
    		next_position(next_position),
    		next_height(next_height),
			area(nullptr),
			increased_ddt(increased_ddt),
			use_min_xy_dist(first.use_min_xy_dist&&second.use_min_xy_dist),
			non_gracious_model_contact(first.non_gracious_model_contact&&second.non_gracious_model_contact)

    	{

			if(first.target_height>second.target_height){
				target_height=first.target_height;
				target_position=first.target_position;
				}
			else{
				target_height=second.target_height;
				target_position=second.target_position;
			}
			effective_radius_height=std::max(first.effective_radius_height,second.effective_radius_height);
			distance_to_top=std::max(first.distance_to_top,second.distance_to_top);

			to_buildplate=first.to_buildplate&&second.to_buildplate;
			to_model_gracious=first.to_model_gracious&&second.to_model_gracious; // valid as we do not merge non gracious with gracious

			AddParents(first.parents);
			AddParents(second.parents);

			elephant_foot_increases=0;
			if(config.diameter_scale_elephant_foot>0){
				coord_t foot_increase_radius = std::abs(std::max(config.getRadius(second),config.getRadius(first))-config.getRadius(*this));
				elephant_foot_increases=foot_increase_radius/(config.branch_radius* (config.diameter_scale_elephant_foot-config.diameter_angle_scale_factor));
			}

    	}

        /*!
         * \brief The layer this support elements wants reach
         */
    	size_t target_height;

        /*!
         * \brief The position this support elements wants to support on layer=target_height
         */
        Point target_position;

        /*!
         * \brief The next position this support elements wants to reach
         */
        Point next_position;


        /*!
         * \brief The next height this support elements wants to reach
         */
        size_t next_height;

        /*!
         * \brief The Effective distance to top of this element regarding radius increases and collision calculations.
         */

        size_t effective_radius_height;

        /*!
         * \brief The element trys to reach the buildplate
         */

        bool to_buildplate;

        /*!
         * \brief All elements in the layer above the current one that are supported by this element
         */
        std::vector<SupportElement*> parents;

        /*!
		 * \brief The amount of layers this element is below the topmost layer of this branch.
		 */

        size_t distance_to_top;

        Polygons* area;

        Point result_on_layer=Point(-1,-1);

        size_t increased_ddt; // how much to model we increased only relevant for merging

        bool to_model_gracious;

        double elephant_foot_increases;

        bool use_min_xy_dist;

        bool non_gracious_model_contact;

        bool operator==(const SupportElement& other) const
        {
            return target_position == other.target_position && target_height==other.target_height;
        }

        bool operator<(const SupportElement& other) const // true if me < other
        {
        	return !(*this==other)&&!(*this>other);
        }
        bool operator> (const SupportElement& other) const{ // doesnt really have to make sense, only required for ordering in maps etc

        	if(*this==other) return false;
        	if(other.target_height!=target_height){
        		return other.target_height<target_height;
        	}

            return other.target_position.X==target_position.X?other.target_position.Y<target_position.Y:other.target_position.X<target_position.X;
        }

        void AddParents(std::vector<SupportElement*> adding){
        	parents.insert(parents.begin(), adding.begin(), adding.end());
        }

    };

    struct TreeSupportSettings{

    	TreeSupportSettings()=default;

    	TreeSupportSettings(const Settings& mesh_group_settings):
    	angle(mesh_group_settings.get<AngleRadians>("support_tree_angle")),
    	angle_slow(mesh_group_settings.get<AngleRadians>("support_tree_angle_slow")),
    	layer_height(mesh_group_settings.get<coord_t>("layer_height")),
    	branch_radius(mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2),
		support_line_width(mesh_group_settings.get<coord_t>("support_line_width")),
		min_radius(1.2*support_line_width/2),
    	maximum_move_distance( (angle < TAU / 4) ? (coord_t) (tan(angle) * layer_height) : std::numeric_limits<coord_t>::max()),
    	maximum_move_distance_slow( (angle_slow < TAU / 4) ? (coord_t) (tan(angle_slow) * layer_height) : std::numeric_limits<coord_t>::max()),
    	support_roof_layers(  mesh_group_settings.get<bool>("support_roof_enable") ?round_divide(mesh_group_settings.get<coord_t>("support_roof_height"),layer_height):0),
    	support_bottom_layers(  mesh_group_settings.get<bool>("support_bottom_enable") ?round_divide(mesh_group_settings.get<coord_t>("support_bottom_height"),layer_height):0),
    	tip_layers(std::max((branch_radius-min_radius) / (support_line_width/3),branch_radius/layer_height)), //ensures lines always stack nicely even if layer height is large
    	diameter_angle_scale_factor(sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius),
    	max_ddt_increase(diameter_angle_scale_factor<=0?std::numeric_limits<coord_t>::max():(mesh_group_settings.get<coord_t>("support_tree_max_radius_increase_by_merges_when_support_to_model"))/(diameter_angle_scale_factor*branch_radius)),
    	min_ddt_to_model(round_up_divide(mesh_group_settings.get<coord_t>("support_tree_min_height_to_model"), layer_height)),
    	increase_radius_until_radius(mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2),
    	increase_radius_until_layer(increase_radius_until_radius<=branch_radius?tip_layers*(increase_radius_until_radius/branch_radius):(increase_radius_until_radius - branch_radius)/(branch_radius*diameter_angle_scale_factor)),
		support_rests_on_model(mesh_group_settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE),
		xy_distance(mesh_group_settings.get<coord_t>("support_xy_distance")),
		bp_radius(mesh_group_settings.get<coord_t>("support_tree_bp_diameter")/2),
		diameter_scale_elephant_foot(std::min(sin(0.7)*layer_height/branch_radius,1.0/(branch_radius / (support_line_width/2)))),
		support_overrides(mesh_group_settings.get<SupportDistPriority>("support_xy_overrides_z")),
		xy_min_distance(support_overrides == SupportDistPriority::Z_OVERRIDES_XY ? mesh_group_settings.get<coord_t>("support_xy_distance_overhang"):xy_distance),
		z_distance_top_layers(round_up_divide(mesh_group_settings.get<coord_t>("support_top_distance"), layer_height)),
		z_distance_bottom_layers(round_up_divide(mesh_group_settings.get<coord_t>("support_bottom_distance"), layer_height)),
		performance_interface_skip_layers(round_up_divide(mesh_group_settings.get<coord_t>("support_interface_skip_height"), layer_height))
    	{
    		layer_start_bp_radius =0;
    		for(coord_t counter=0;branch_radius+branch_radius*counter*diameter_scale_elephant_foot<=bp_radius;counter++){
    			layer_start_bp_radius=counter;
    		}
    		//for performance reasons it is not possible to set a VERY low xy_distance or xy_min_distance (<0.1mm)
    		//as such if such a value is set the support will be outset by .1mm to ensure all points are supported that should be.
    		//This is not the best solution, but the only one to ensure areas can not lag though walls at high maximum_move_distance.
    		if(xy_distance<100){
    			xy_distance=100;
    		}
    		if(xy_min_distance<100){
				xy_min_distance=100;
				performance_increased_xy_min=true;
			}

    	}

    private:

		double angle;
		double angle_slow;

    public:



		coord_t layer_height;
    	coord_t branch_radius;
		coord_t support_line_width; // only used for some smoothing, for the logic mainly irrelevant
		coord_t min_radius;
		coord_t maximum_move_distance;
		coord_t maximum_move_distance_slow;
		size_t support_roof_layers;
		size_t support_bottom_layers;
		size_t tip_layers;
		double diameter_angle_scale_factor;
		size_t max_ddt_increase;
		size_t min_ddt_to_model;
		coord_t increase_radius_until_radius;
		size_t increase_radius_until_layer;
		bool support_rests_on_model;
		coord_t xy_distance;
		coord_t bp_radius;
		coord_t layer_start_bp_radius;
		double diameter_scale_elephant_foot;
		SupportDistPriority support_overrides;
		coord_t xy_min_distance;
		size_t z_distance_top_layers;
		size_t z_distance_bottom_layers;
		size_t performance_interface_skip_layers;// only relevant at the bottom
		bool performance_increased_xy_min=false;
    public:

		bool hasSameInfluenceAreaSettings(const TreeSupportSettings& other) const {
				return 	branch_radius == other.branch_radius &&
						tip_layers == other.tip_layers &&
						diameter_angle_scale_factor == other.diameter_angle_scale_factor &&
						layer_start_bp_radius == other.layer_start_bp_radius &&
						bp_radius == other.bp_radius &&
						diameter_scale_elephant_foot == other.diameter_scale_elephant_foot &&
						min_radius == other.min_radius &&
						xy_min_distance == other.xy_min_distance && // as a recalculation of the AvoidanceAreas is required to set a new min_radius. This is not necessary if only xy_distance changes
						support_rests_on_model == other.support_rests_on_model &&
						increase_radius_until_layer == other.increase_radius_until_layer &&
						min_ddt_to_model == other.min_ddt_to_model &&
						max_ddt_increase == other.max_ddt_increase &&
						maximum_move_distance == other.maximum_move_distance &&
						maximum_move_distance_slow == other.maximum_move_distance_slow&&
						z_distance_bottom_layers == other.z_distance_bottom_layers &&
						support_line_width == other.support_line_width&&
						support_overrides == other.support_overrides; //requires new avoidance calculation. Could be changed so it does not, but because i expect that this case happens seldom i dont think it is worth it.
		}
		bool hasSameAvoidanceSettings(const TreeSupportSettings& other) const {
				return  z_distance_bottom_layers == other.z_distance_bottom_layers &&
						z_distance_top_layers ==other.z_distance_top_layers &&
						support_overrides == other.support_overrides &&
						branch_radius == other.branch_radius &&
						tip_layers == other.tip_layers &&
						diameter_angle_scale_factor == other.diameter_angle_scale_factor &&
						layer_start_bp_radius == other.layer_start_bp_radius &&
						bp_radius == other.bp_radius &&
						diameter_scale_elephant_foot == other.diameter_scale_elephant_foot &&
						min_radius == other.min_radius &&
						xy_min_distance == other.xy_min_distance &&
						maximum_move_distance == other.maximum_move_distance &&
						maximum_move_distance_slow == other.maximum_move_distance_slow;
		}



	    /*!
	     * \brief Get the Distance to top regarding the real radius this part will have. This is different from distance_to_top, which is can be used to calculate the top most layer of the branch.
	     * \param elem[in] The SupportElement one wants to know the effectiveDTT
	     * \return The Effective DTT.
	     */
    	inline size_t getEffektiveDDT(const TreeSupport::SupportElement& elem) const { // can be bigger if we use bp radius
    		return elem.effective_radius_height<increase_radius_until_layer ? (elem.distance_to_top<increase_radius_until_layer ? elem.distance_to_top : increase_radius_until_layer) : elem.effective_radius_height ;
    	}

	    /*!
	     * \brief Get the Radius part will have based on numeric values.
	     * \param distance_to_top[in] The effective distance_to_top of the element
	     * \param elephant_foot_increases[in] The elephant_foot_increases of the element.
	     * \return The radius an element with these attributes would have.
	     */
    	inline coord_t getRadius(size_t distance_to_top,const double elephant_foot_increases=0)const {
    		return  (distance_to_top<=tip_layers ?min_radius+(branch_radius-min_radius) * distance_to_top/ tip_layers : // tip
    				branch_radius +  //base
					branch_radius * (distance_to_top-tip_layers) * diameter_angle_scale_factor)+  // gradual increase
					branch_radius * elephant_foot_increases * (std::max(diameter_scale_elephant_foot-diameter_angle_scale_factor,0.0));
    	}

	    /*!
	     * \brief Get the Radius, that this element will have.
	     * \param elem[in] The Element.
	     * \return The radius the element has.
	     */
    	inline coord_t getRadius(const TreeSupport::SupportElement& elem) const {

    		return getRadius(getEffektiveDDT(elem),elem.elephant_foot_increases);
    	}

	    /*!
	     * \brief Get the collision Radius of this Element. This can be smaller then the actual radius, as the drawAreas will cut off areas that may collide with the model.
	     * \param elem[in] The Element.
	     * \return The collision radius the element has.
	     */
    	inline coord_t getCollisionRadius(const TreeSupport::SupportElement& elem) const {

    		return getRadius(elem.effective_radius_height,elem.elephant_foot_increases);
    	}

	    /*!
	     * \brief Get the Radius an element should at least have at a given layer.
	     * \param layer_nr[in] The layer.
	     * \return The radius every element should aim to achieve.
	     */
    	inline coord_t reccomendedMinRadius(coord_t layer_nr) const {
    		double scale=(layer_start_bp_radius-layer_nr) * diameter_scale_elephant_foot;
    		return scale>0?branch_radius+branch_radius*scale:0;
    	}

    };

private:
	enum class LineStatus{INVALID,TO_MODEL,TO_MODEL_GRACIOUS,TO_BP};
	using LineInformation=std::vector<std::pair<Point,TreeSupport::LineStatus>>;

	coord_t precalculate(SliceDataStorage& storage,std::vector<size_t> currently_processing_meshes);
    std::vector<LineInformation> convertLinesToInternal(Polygons polylines,coord_t layer_nr);
    Polygons convertInternalToLines(std::vector<TreeSupport::LineInformation> lines);

    std::pair<std::vector<LineInformation>,std::vector<LineInformation>> splitLines(std::vector<LineInformation> lines,size_t current_layer,size_t ddt);//assumes all Points on the current line are valid

    Polygons ensureMaximumDistancePolyline (const Polygons& input, coord_t distance,size_t min_points)const;
    Polygons toPolylines(const Polygons& poly)const;



    //generates Points where the Model should be supported and creates the areas where these points have to be placed

    void generateInitalAreas(const SliceMeshStorage& mesh,std::vector<std::map<SupportElement*,Polygons*>>& move_bounds,SliceDataStorage& storage);

    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This helper merges all elements in input with the elements into reduced_new_layer.
     * Elements in input are merged together if possible, while elements reduced_new_layer are not checked against each other.
     *
     * \param reduced_new_layer[in,out] The processed elements. Value is the outermost area where the wall of the later printed support could be.
     * \param reduced_new_layer_aabb[in,out] Axis-aligned Boundary Boxes of the processed elements
     * \param input[in] Not yet processed elements
     * \param input_aabb[in] Axis-aligned Boundary Boxes of the not yet processed elements. These do not have to be present, and are calculated if not provided.
     * \param main[in] The Elements of the current Layer that will reach the buildplate. Value is the influence area where the center of a circle of support may be placed.
     * \param secondary[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
     * Value is the influence area where the center of a circle of support may be placed.
     * \param insertMain[out] Elements to be inserted into the main dictionary after the Helper terminates.
     * \param insertSecondary[out] Elements to be inserted into the secondary dictionary after the Helper terminates.
     */
    void mergeHelper(std::map<SupportElement,Polygons>& reduced_new_layer,
    		std::map<SupportElement,AABB>& reduced_new_layer_aabb,
			std::map<SupportElement,Polygons>& input,
			std::map<SupportElement,AABB>& input_aabb,
    		const std::unordered_map<SupportElement,Polygons>& main,
    		const std::unordered_map<SupportElement,Polygons>& secondary,
			std::unordered_map<SupportElement,Polygons>& insert_main,
			std::unordered_map<SupportElement,Polygons>& insert_secondary,
    		std::vector<SupportElement>& erase,
    		coord_t layer_nr);
    /*!
     * \brief Merges Influence Areas if possible.
     *
     * Branches which do overlap have to be merged. This manages the helper and uses a divide and conquer approach to parallelize this problem.
     *
     * \param input[in] Value is the outermost area where the wall of the later printed support could be.
     * \param main[in] The Elements of the current Layer that will reach the buildplate.
     *  Value is the influence area where the center of a circle of support may be placed.
     * \param secondary[in] The Elements of the current Layer that do not have to reach the buildplate. Also contains main as every element that can reach the buildplate is not forced to.
     *  Value is the influence area where the center of a circle of support may be placed.
     */
    void mergePolygonsDaQ(std::map<SupportElement,Polygons>& input,std::unordered_map<SupportElement,Polygons>& main,std::unordered_map<SupportElement,Polygons>& secondary,int layer_nr);
    /*!
     * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
     * \param me[in] Polygons object that has to be offsetted.
     * \param distance[in] The distance by which me should be offsetted. Expects values >=0.
     * \param collision[in] The area representing obstacles.
     * \param last_safe_step_size[in] The most it is allowed to offset in one step.
     * \param min_amount_offset[in] How many steps have to be done at least. As this uses round offset this increases the ammount of vertecies, which may be required if Polygons get very small.
     * \return The resulting Polygons object.
     */
    Polygons safeOffsetInc(Polygons& me,coord_t distance,const Polygons& collision,coord_t last_safe_step_size,size_t min_amount_offset)const;

    /*!
     * \brief Increases influence areas as far as required.
     *
     * Calculates influence areas of the layer below, based on the influence areas of the current layer.
     * Increases every influence area by maximum_move_distance_slow. If this is not enough, as in we would change our gracious or to_buildplate status the influence areas are instead increased by maximum_move_distance_slow.
     * Also ensures that increasing the radius of a branch, does not cause it to change its status (like to_buildplate ). If this were the case, the radius is not increased instead.
     *
     * Warning: The used format inside this is different as the SupportElement does not have a valid area member. Instead this area is saved as value of the dictionary. This was done to avoid not needed heap allocations.
     *
     * \param new_layer[out] Influence areas that can reach the buildplate
     * \param new_layer_merge[out] A map of each SupportElement to the area the support could use. This is the influence area offsetted by the radius. This is needed to properly merge influence areas.
     * \param new_layer_to_model[in] Influence areas that do not have to reach the buildplate. This has overlap with new_layer param, as areas that can reach the buildplate also can reach the model.
     * This redundency is required if a to_buildplate influence area is allowed to merge with a to model influence area.
     * \param new_layer_bypass_merge[out] Influence areas ready to be added to the layer below that do not need merging.
     * \param last_layer[in] Influence areas of the current layer.
     * \param layer_nr[in] Number of the current layer.
     */
    void increaseAreas(std::unordered_map<SupportElement,Polygons>& new_layer,std::map<SupportElement,Polygons>& new_layer_merge,std::unordered_map<SupportElement,Polygons>& new_layer_to_model, std::vector<std::pair<SupportElement*,Polygons*>>& new_layer_bypass_merge, const std::vector<std::pair<SupportElement*,Polygons*>>& last_layer,size_t layer_nr);

    /*!
     * \brief Propergates influence downwards, and merges overlapping ones.
     *
     * \param move_bounds[in,out] All currently existing influence areas
     */
    void createLayerPathing(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds);


    /*!
     * \brief Sets the result_on_layer for all parents based on the SupportElement supplied.
     *
     * \param elem[in] The SupportElements, which parents position should be determined.
     */
    void setPointsOnAreas(const SupportElement* elem);
    /*!
     * \brief Get the best point to connect to the model and set the result_on_layer of the relevant SupportElement accordingly.
     *
     * \param move_bounds[in,out] All currently existing influence areas
     * \param first_elem[in,out] SupportElement that did not have its result_on_layer set meaning that it does not have a child element.
     * \param layer_nr[in] The current layer.
     */
    bool setToModelContact(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds, SupportElement* first_elem ,const size_t layer_nr);

    /*!
     * \brief Set the result_on_layer point for all influence areas
     *
     * \param move_bounds[in,out] All currently existing influence areas
     */
    void createNodesFromArea(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds);

    /*!
     * \brief Draws circles around resul_on_layer points of the influence areas
     *
     * \param move_bounds[in,out] All currently existing influence areas
     * \param storage[out ? todo] The storage where the support should be stored.
     */
    void drawAreas(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds,SliceDataStorage& storage);


    std::vector<std::pair<TreeSupportSettings,std::vector<size_t>>> grouped_meshes;
    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes
     *
     */
    ModelVolumes volumes_;
    /*!
     * \brief Contains support areas that are ready to be added to storage later.
     */
    std::vector<Polygons> precalculated_support_layers;

    /*!
     * \brief Contains config settings to avoid loading them in every function. This was done to improve readability of the code.
     */
    TreeSupportSettings config;

    double progress_multiplier=1;
    double progress_offset=0;

};


}

namespace std
{
    template<> struct hash<cura::TreeSupport::SupportElement>
    {
        size_t operator()(const cura::TreeSupport::SupportElement& node) const
        {
            return hash<cura::Point>()(node.target_position) +node.target_height;
        }
    };
}

#endif /* TREESUPPORT_H */
