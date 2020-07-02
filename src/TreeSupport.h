//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TREESUPPORT_H
#define TREESUPPORT_H

#include <forward_list>
#include <unordered_set>
#include "settings/EnumSettings.h"

namespace cura
{
/*!
 * \brief Lazily generates tree guidance volumes.
 *
 * \warning This class is not currently thread-safe and should not be accessed in OpenMP blocks
 */
class ModelVolumes
{
public:
    ModelVolumes() = default;
    /*!
     * \brief Construct the ModelVolumes object
     *
     * \param storage The slice data storage object to extract the model
     * contours from.
     * \param xy_distance The required clearance between the model and the
     * tree branches.
     * \param max_move The maximum allowable movement between nodes on
     * adjacent layers
     * \param radius_sample_resolution Sample size used to round requested node radii.
     */
    ModelVolumes(const SliceDataStorage& storage, coord_t xy_distance, coord_t max_move,coord_t max_move_slow ,coord_t radius_sample_resolution,
    		coord_t z_distance_bottom_layers,coord_t z_distance_top_layers,bool support_rests_on_model,bool avoid_support_blocker,bool use_legacy_tree_support,bool use_exponential_collision_resolution,coord_t exponential_threashold,double exponential_factor);

    ModelVolumes(ModelVolumes&&) = default;
    ModelVolumes& operator=(ModelVolumes&&) = default;

    ModelVolumes(const ModelVolumes&) = delete;
    ModelVolumes& operator=(const ModelVolumes&) = delete;

    /*!
     * \brief Creates the areas that have to be avoided by the tree's branches.
     *
     * The result is a 2D area that would cause nodes of radius \p radius to
     * collide with the model.
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& getCollision(coord_t radius, LayerIndex layer_idx) const;

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
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */

    const Polygons& getAvoidance(coord_t radius, LayerIndex layer_idx,bool slow=false,bool to_model=false) const;

    /*!
     * \brief Generates the area of a given layer that must be avoided if the
     * branches wish to go towards the model
     *
     * The area represents the areas that do not collide with the model but
     * are unable to reach the build platform
     *
     * \param radius The radius of the node of interest
     * \param layer The layer of interest
     * \return Polygons object
     */
    const Polygons& getInternalModel(coord_t radius, LayerIndex layer_idx) const;
    const Polygons& getPlaceableAreas(coord_t radius, LayerIndex layer_idx) const;

    void precalculateAvoidance(const size_t maxLayer,const coord_t branch_radius, const double scaleFactor ,const coord_t min_max_radius,const double scale_foot ); //__attribute__((optimize("-O3")))

    coord_t getMaxMove(){
    	return max_move_;
    }

    const std::vector<coord_t> getRadii(){
    	return resolution_steps;
    }
    /*!
     * \brief Round \p radius upwards to a multiple of radius_sample_resolution_
     *
     * \param radius The radius of the node of interest
     */
    coord_t ceilRadius(coord_t radius) const;
private:
    /*!
     * \brief Convenience typedef for the keys to the caches
     */
    using RadiusLayerPair = std::pair<coord_t, LayerIndex>;


    /*!
     * \brief Calculate the collision areas at the radius and layer indicated
     * by \p key.
     *
     * \param key The radius and layer of the node of interest
     */
    const Polygons& calculateCollision(const RadiusLayerPair& key) const;

    /*!
     * \brief Calculate the avoidance areas at the radius and layer indicated
     * by \p key.
     *
     * \param key The radius and layer of the node of interest
     */
    const Polygons& calculateAvoidance(const RadiusLayerPair& key) const;

    /*!
     * \brief Calculate the internal model areas at the radius and layer
     * indicated by \p key.
     *
     * \param key The radius and layer of the node of interest
     */
    const Polygons& calculateInternalModel(const RadiusLayerPair& key) const;

    /*!
     * \brief Calculate the collision area around the printable area of the machine.
     *
     * \param a Polygons object representing the non-printable areas on and around the build platform
     */
    static Polygons calculateMachineBorderCollision(Polygon machine_border);

    /*!
     * \brief Polygons representing the limits of the printable area of the
     * machine
     */
    Polygons machine_border_;

    /*!
     * \brief The required clearance between the model and the tree branches
     */
    coord_t xy_distance_;

    /*!
     * \brief The maximum distance that the centrepoint of a tree branch may
     * move in consequtive layers
     */
    coord_t max_move_;

    /*!
     * \brief The maximum distance that the centrepoint of a tree branch wants to
     * move in consequtive layers (slower than max_move)
     */
    coord_t max_move_slow;

    /*!
     * \brief Sample resolution for radius values.
     *
     * The radius will be rounded (upwards) to multiples of this value before
     * calculations are done when collision, avoidance and internal model
     * Polygons are requested.
     */
    coord_t radius_sample_resolution_;


    coord_t z_distance_bottom_layers;

    coord_t z_distance_top_layers;


    bool support_rests_on_model;


    bool avoid_support_blocker;
    bool use_legacy_tree_support;
    bool use_exponential_collision_resolution;
    coord_t exponential_threashold;
    double exponential_factor;

    /*!
     * \brief Storage for layer outlines of the meshes.
     */
    std::vector<Polygons> layer_outlines_;
    std::vector<Polygons> anti_overhang_;

    std::vector<coord_t> resolution_steps=std::vector<coord_t>();

    /*!
     * \brief Caches for the collision, avoidance and internal model polygons
     * at given radius and layer indices.
     *
     * These are mutable to allow modification from const function. This is
     * generally considered OK as the functions are still logically const
     * (ie there is no difference in behaviour for the user betweeen
     * calculating the values each time vs caching the results).
     */
    mutable std::unordered_map<RadiusLayerPair, Polygons> collision_cache_;
    mutable long collision_cache_hit=0;
    mutable long collision_cache_miss=0;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_slow;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_to_model;
    mutable std::unordered_map<RadiusLayerPair, Polygons> avoidance_cache_to_model_slow;
    mutable std::unordered_map<RadiusLayerPair, Polygons> placeable_areas_cache;
    mutable std::unordered_map<RadiusLayerPair, Polygons> internal_model_cache_;


};

class SliceDataStorage;
class SliceMeshStorage;


//saves all the setting for easy access and without needing 20 lines getting them from the main Settings object at the start of every function




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

    coord_t precalculate(SliceDataStorage &storage);


    /*!
     * \brief Represents the metadata of a node in the tree.
     */
    struct Node
    {
        static constexpr Node* NO_PARENT = nullptr;

        Node()
         : distance_to_top(0)
         , position(Point(0, 0))
         , skin_direction(false)
         , support_roof_layers_below(0)
         , to_buildplate(true)
         , parent(nullptr)
        {}

        Node(const Point position, const size_t distance_to_top, const bool skin_direction, const int support_roof_layers_below, const bool to_buildplate, Node* const parent)
         : distance_to_top(distance_to_top)
         , position(position)
         , skin_direction(skin_direction)
         , support_roof_layers_below(support_roof_layers_below)
         , to_buildplate(to_buildplate)
         , parent(parent)
        {}

#ifdef DEBUG // Clear the delete node's data so if there's invalid access after, we may get a clue by inspecting that node.
        ~Node()
        {
        	delete nextTarget; // TODO KEEP ALWAYS
            parent = nullptr;
            merged_neighbours.clear();
        }
#endif // DEBUG

        size_t target_height=0;

        // where to move next
        Polygons* nextTarget=new Polygons();

        /*!
         * \brief The number of layers to go to the top of this branch.
         */
        size_t distance_to_top;

        /*!
         * \brief The position of this node on the layer.
         */
        Point position;

        /*!
         * \brief The direction of the skin lines above the tip of the branch.
         *
         * This determines in which direction we should reduce the width of the
         * branch.
         */
        bool skin_direction;

        /*!
         * \brief The number of support roof layers below this one.
         *
         * When a contact point is created, it is determined whether the mesh
         * needs to be supported with support roof or not, since that is a
         * per-mesh setting. This is stored in this variable in order to track
         * how far we need to extend that support roof downwards.
         */
        int support_roof_layers_below;

        /*!
         * \brief Whether to try to go towards the build plate.
         *
         * If the node is inside the collision areas, it has no choice but to go
         * towards the model. If it is not inside the collision areas, it must
         * go towards the build plate to prevent a scar on the surface.
         */
        bool to_buildplate;

        /*!
         * \brief The originating node for this one, one layer higher.
         *
         * In order to prune branches that can't have any support (because they
         * can't be on the model and the path to the buildplate isn't clear),
         * the entire branch needs to be known.
         */
        Node *parent;

        /*!
        * \brief All neighbours (on the same layer) that where merged into this node.
        *
        * In order to prune branches that can't have any support (because they
        * can't be on the model and the path to the buildplate isn't clear),
        * the entire branch needs to be known.
        */
        std::forward_list<Node*> merged_neighbours;

        bool operator==(const Node& other) const
        {
            return position == other.position;
        }
    };
    struct TreeSupportSettings; // forward declaration as we need some config values in the merge case


    struct SupportElement{

    	SupportElement(coord_t distance_to_top,size_t target_height,Point target_position,bool to_buildplate,bool to_model_gracious): //SupportElement(target_height,target_position,target_height,target_position,distance_to_top,distance_to_top,to_buildplate)
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
			elephant_foot_increases(0)

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
				elephant_foot_increases(elem.elephant_foot_increases)

    	{
    		parents.insert(parents.begin(), elem.parents.begin(), elem.parents.end());
    	}

    	SupportElement(const SupportElement& first,SupportElement* newParent):
    			target_height(first.target_height),
    			target_position(first.target_position),
    			next_position(first.next_position),
    			next_height(first.next_height),
    			effective_radius_height(first.effective_radius_height),
    			to_buildplate(first.to_buildplate),
				distance_to_top(first.distance_to_top+1),
				area(first.area),
				result_on_layer(Point(-1,-1)), // set to invalid as we are a new node on a new layer
				increased_ddt(first.increased_ddt),
				to_model_gracious(first.to_model_gracious),
				elephant_foot_increases(first.elephant_foot_increases)
    	{
    		parents={newParent};
    	}

    	SupportElement(const SupportElement& first,const SupportElement& second,size_t next_height,Point next_position,coord_t increased_ddt,TreeSupportSettings config):
    		next_position(next_position),
    		next_height(next_height),
			area(nullptr),
			increased_ddt(increased_ddt)

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
			// extra rad = branch_radius * elephant_foot_increases * (diameter_scale_elephant_foot-diameter_angle_scale_factor) => ele_inc =extraRad/(branch_radius* (diameter_scale_elephant_foot-diameter_angle_scale_factor))
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
         * \brief The Effective distance to top of this element regarding radius increases and collsision calculations.
         */

        size_t effective_radius_height;

        /*!
         * \brief The element trys to reach the buildplate
         */

        bool to_buildplate;

        std::vector<SupportElement*> parents;

        size_t distance_to_top;

        Polygons* area;

        Point result_on_layer=Point(-1,-1);

        size_t increased_ddt; // how much to model we increased only relevant for merging

        bool to_model_gracious;

        double elephant_foot_increases;


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

    	TreeSupportSettings(const Settings& mesh_group_settings):branch_radius(mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2),
    	layer_height(mesh_group_settings.get<coord_t>("layer_height")),
    	angle(mesh_group_settings.get<AngleRadians>("support_tree_angle")),
    	angle_slow(mesh_group_settings.get<AngleRadians>("support_tree_angle_slow")),
    	maximum_move_distance( (angle < TAU / 4) ? (coord_t) (tan(angle) * layer_height) : std::numeric_limits<coord_t>::max()),
    	maximum_move_distance_slow( (angle_slow < TAU / 4) ? (coord_t) (tan(angle_slow) * layer_height) : std::numeric_limits<coord_t>::max()),
    	support_roof_layers(  mesh_group_settings.get<bool>("support_roof_enable") ?round_divide(mesh_group_settings.get<coord_t>("support_roof_height"),layer_height):0),
    	tip_layers(branch_radius / layer_height),
    	diameter_angle_scale_factor(sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius),
    	max_ddt_increase(diameter_angle_scale_factor<=0?std::numeric_limits<coord_t>::max():(mesh_group_settings.get<coord_t>("support_tree_max_radius_increase_by_merges_when_support_to_model"))/(diameter_angle_scale_factor*branch_radius)),
    	min_ddt_to_model(round_up_divide(mesh_group_settings.get<coord_t>("support_tree_min_height_to_model"), layer_height)),
    	increase_radius_until_radius(mesh_group_settings.get<coord_t>("support_tree_increase_radius_until_radius")),
    	increase_radius_until_layer(increase_radius_until_radius<=branch_radius?tip_layers*(increase_radius_until_radius/branch_radius):(increase_radius_until_radius - branch_radius)/(branch_radius*diameter_angle_scale_factor)),
		dont_move_until_ddt(round_up_divide(mesh_group_settings.get<coord_t>("support_tree_dont_move_distance"), layer_height)),
		support_rests_on_model(mesh_group_settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE),
		xy_distance(mesh_group_settings.get<coord_t>("support_xy_distance")),
		bp_radius(mesh_group_settings.get<coord_t>("support_tree_bp_radius")),
		diameter_scale_elephant_foot(sin(1.04)*layer_height/branch_radius),
		support_line_width(mesh_group_settings.get<coord_t>("support_line_width"))
    	{
    		layer_start_bp_radius =0;
    		for(coord_t counter=0;branch_radius+branch_radius*counter*diameter_scale_elephant_foot<=bp_radius;counter++){
    			layer_start_bp_radius=counter;
    		}
    	}

    	coord_t branch_radius;
		coord_t layer_height;
		double angle;
		double angle_slow;
		coord_t maximum_move_distance;
		coord_t maximum_move_distance_slow;
		size_t support_roof_layers;
		size_t tip_layers;
		double diameter_angle_scale_factor;
		size_t max_ddt_increase;
		size_t min_ddt_to_model;
		coord_t increase_radius_until_radius;
		size_t increase_radius_until_layer;;
		size_t dont_move_until_ddt;
		bool support_rests_on_model;
		coord_t xy_distance;
		coord_t bp_radius;
		coord_t layer_start_bp_radius;
		double diameter_scale_elephant_foot;
		coord_t support_line_width;
    public:

    	size_t getEffektiveDDT(const TreeSupport::SupportElement& elem) const { // can be bigger if we use bp radius
    		return elem.effective_radius_height<increase_radius_until_layer ? (elem.distance_to_top<increase_radius_until_layer ? elem.distance_to_top : increase_radius_until_layer) : elem.effective_radius_height ;
    	}

    	coord_t getRadius(const size_t distance_to_top,const double elephant_foot_increases=0)const {

    		if(distance_to_top==1) // todo integrate properly into formula
    			return support_line_width*1.15/2;
    		if(distance_to_top==2)
    			return support_line_width*2.5/2;


    		return  distance_to_top<=tip_layers ?branch_radius * distance_to_top/ tip_layers : // tip
    				branch_radius +  //base
					branch_radius * (distance_to_top-tip_layers) * diameter_angle_scale_factor+  // gradual increase
					branch_radius * elephant_foot_increases * (std::max(diameter_scale_elephant_foot-diameter_angle_scale_factor,0.0));
    	}
    	coord_t getRadius(const TreeSupport::SupportElement& elem) const {

    		return getRadius(getEffektiveDDT(elem),elem.elephant_foot_increases);
    	}

    	coord_t getCollisionRadius(const TreeSupport::SupportElement& elem) const {

    		return getRadius(elem.effective_radius_height,elem.elephant_foot_increases);
    	}


    	coord_t reccomendedMinRadius(coord_t layer_nr) const {
    		double scale=(layer_start_bp_radius-layer_nr) * diameter_scale_elephant_foot;
    		return scale>0?branch_radius+branch_radius*scale:0;
    	}

    };

private:
	enum LineStatus{INVALID,TO_MODEL,TO_MODEL_GRACIOUS,TO_BP};
	using LineInformation=std::vector<std::pair<Point,TreeSupport::LineStatus>>;
    /*!
     * \brief Generator for model collision, avoidance and internal guide volumes
     *
     * Lazily computes volumes as needed.
     *  \warning This class is NOT currently thread-safe and should not be accessed in OpenMP blocks
     */
    ModelVolumes volumes_;
    std::vector<Polygons> precalculatedSupportLayers;


    TreeSupportSettings config;


    /*!
     * \brief Draws circles around each node of the tree into the final support.
     *
     * This also handles the areas that have to become support roof, support
     * bottom, the Z distances, etc.
     *
     * \param storage[in, out] The settings storage to get settings from and to
     * save the resulting support polygons to.
     * \param contact_nodes The nodes to draw as support.
     */
    void drawCircles(SliceDataStorage& storage, const std::vector<std::unordered_set<Node*>>& contact_nodes);

    /*!
     * \brief Drops down the nodes of the tree support towards the build plate.
     *
     * This is where the cleverness of tree support comes in: The nodes stay on
     * their 2D layers but on the next layer they are slightly shifted. This
     * causes them to move towards each other as they are copied to lower layers
     * which ultimately results in a 3D tree.
     *
     * \param contact_nodes[in, out] The nodes in the space that need to be
     * dropped down. The nodes are dropped to lower layers inside the same
     * vector of layers.
     */
    void dropNodes(std::vector<std::unordered_set<Node*>>& contact_nodes);

    std::vector<LineInformation> convertLinesToInternal(Polygons polylines,coord_t layer_nr);
    Polygons convertInternalToLines(std::vector<TreeSupport::LineInformation> lines);

    std::pair<std::vector<LineInformation>,std::vector<LineInformation>> splitLines(std::vector<LineInformation> lines,size_t current_layer,size_t ddt);//assumes all Points on the current line are valid
    std::pair<Polygons,Polygons> splitLines(Polygons lines,size_t current_layer,size_t ddt,LineStatus keepUntil);

    //generates Points where the Model should be supported and creates the areas where these points have to be placed

    void generateInitalAreas(const SliceMeshStorage &mesh,std::vector<std::map<SupportElement*,Polygons*>>& moveBounds,SliceDataStorage &storage);

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
			std::unordered_map<SupportElement,Polygons>& insertMain,
			std::unordered_map<SupportElement,Polygons>& insertSecondary,
    		std::vector<SupportElement>& erase,
    		coord_t layer_nr)const;
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
    void mergePolygonsDaQ(std::map<SupportElement,Polygons>& input,std::unordered_map<SupportElement,Polygons>& main,std::unordered_map<SupportElement,Polygons>& secondary,int layer_nr) const;
    Polygons safeOffsetInc(Polygons& me,coord_t distance,const Polygons& collision,coord_t lastSafeStepSize=0,size_t min_amount_offset=0)const;


    void increaseAreas(std::unordered_map<SupportElement,Polygons>& newLayer,std::map<SupportElement,Polygons>& newLayer_merge,std::unordered_map<SupportElement,Polygons>& newLayer_to_model,std::vector<std::pair<SupportElement*,Polygons*>> lstLayer,size_t layer_nr);
    void createLayerPathing(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds);
    void setPointsOnAreas(SupportElement* elem, coord_t max_move);
    bool setToModelContact(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds, SupportElement* firstElem ,const size_t layer_nr);

    void createNodesFromArea(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds);
    void saveSliceAsSvg(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds,const SliceDataStorage &storage);

    void drawAreas(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds,SliceDataStorage &storage);

    /*!
     * \brief Creates points where support contacts the model.
     *
     * A set of points is created for each layer.
     * \param mesh The mesh to get the overhang areas to support of.
     * \param contact_nodes[out] A vector of mappings from contact points to
     * their tree nodes.
     * \param collision_areas For every layer, the areas where a generated
     * contact point would immediately collide with the model due to the X/Y
     * distance.
     * \return For each layer, a list of points where the tree should connect
     * with the model.
     */
    void generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<Node*>>& contact_nodes);

    /*!
     * \brief Add a node to the next layer.
     *
     * If a node is already at that position in the layer, the nodes are merged.
     */
    void insertDroppedNode(std::unordered_set<Node*>& nodes_layer, Node* node);
};


}

namespace std
{
    template<> struct hash<cura::TreeSupport::Node>
    {
        size_t operator()(const cura::TreeSupport::Node& node) const
        {
            return hash<cura::Point>()(node.position);
        }
    };
    template<> struct hash<cura::TreeSupport::SupportElement>
    {
        size_t operator()(const cura::TreeSupport::SupportElement& node) const
        {
            return hash<cura::Point>()(node.target_position) +node.target_height;
        }
    };
}

#endif /* TREESUPPORT_H */
