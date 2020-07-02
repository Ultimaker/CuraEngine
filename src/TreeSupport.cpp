//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Slice.h"
#include "sliceDataStorage.h"
#include "TreeSupport.h"
#include "progress/Progress.h"
#include "settings/EnumSettings.h"
#include "settings/types/AngleRadians.h" //Creating the correct branch angles.
#include "settings/types/Ratio.h"
#include "utils/IntPoint.h" //To normalize vectors.
#include "utils/logoutput.h"
#include "utils/math.h" //For round_up_divide and PI.
#include "utils/MinimumSpanningTree.h" //For connecting the correct nodes together to form an efficient tree.
#include "utils/polygon.h" //For splitting polygons into parts.
#include "utils/polygonUtils.h" //For moveInside.
#include <stdio.h>
#include <chrono>
#include <omp.h>
#include "infill.h"
#include "utils/linearAlg2D.h"

#include <string>
#include <numeric>
#include <fstream>

#define SQRT_2 1.4142135623730950488 //Square root of 2.
#define CIRCLE_RESOLUTION 10 //The number of vertices in each circle.

//The various stages of the process can be weighted differently in the progress bar.
//These weights are obtained experimentally.
#define PROGRESS_WEIGHT_DROPDOWN 50 //Dropping down support.
#define PROGRESS_WEIGHT_AREAS 1 //Creating support areas.
//These weights are obtained experimentally.
#define PROGRESS_PRECALC_COLL 1000
#define PROGRESS_PRECALC_AVO 6000
#define PROGRESS_GENERATE_NODES 1000
#define PROGRESS_AREA_CALC 2000
#define PROGRESS_TOTAL 10000

namespace cura {

void loadDemoConfig(Settings& mesh_group_settings){ //TODO REMOVE ONLY FOR DEMO
	std::ifstream infile("demo.cfg");
	if(!infile.good()){
		std::ofstream outfile("demo.cfg");
		outfile << "support_tree_angle_slow 50\nsupport_tree_max_radius_increase_by_merges_when_support_to_model 0.5\nsupport_tree_min_height_to_model 1.0\nsupport_tree_increase_radius_until_radius 2.0\nsupport_tree_dont_move_distance 0.5\nsupport_tree_bp_radius 7.5\nsupport_tree_only_gracious_to_model false\nsupport_tree_use_smart_wall true\n    support_tree_support_line_outset 0.6\n    support_tree_easy_remove true\nsupport_tree_contact_node_pattern lines\nsupport_tree_contact_line_distance 5.0\nsupport_tree_avoid_support_blocker true\nsupport_tree_use_legacy false\nsupport_tree_internal_use_exponential_collision_resolution true\n    support_tree_internal_exponential_threashold 1\n    support_tree_internal_exponential_factor 1.25\n";
		outfile.close();
		infile=std::ifstream("demo.cfg");
	}
	std::string line;
	std::string value;
	while (infile >> line >> value)
	{
		logAlways("Loaded demo Key: %s with Value: %s\n",line.c_str(),value.c_str());
		mesh_group_settings.add(line,value);
	}
}


TreeSupport::TreeSupport(const SliceDataStorage &storage) {

	Settings& mesh_group_settings =
			Application::getInstance().current_slice->scene.current_mesh_group->settings;
	loadDemoConfig(mesh_group_settings);
	const coord_t radius_sample_resolution = mesh_group_settings.get<coord_t>(
			"support_tree_collision_resolution");
	config=TreeSupportSettings(mesh_group_settings);
	const size_t z_distance_bottom_layers = round_up_divide(mesh_group_settings.get<coord_t>("support_bottom_distance"),config.layer_height);
	coord_t z_distance_top_layers= round_up_divide(mesh_group_settings.get<coord_t>("support_top_distance"),config.layer_height);
	precalculatedSupportLayers=std::vector<Polygons>(storage.support.supportLayers.size());
	const bool avoid_support_blocker=mesh_group_settings.get<bool>("support_tree_avoid_support_blocker");
	const bool use_legacy_tree_support=mesh_group_settings.get<bool>("support_tree_use_legacy");
	const bool use_exponential_collission_resolution=mesh_group_settings.get<bool>("support_tree_internal_use_exponential_collision_resolution");
	const double exponential_factor=mesh_group_settings.get<double>("support_tree_internal_exponential_factor");
	const coord_t exponential_threashold=mesh_group_settings.get<coord_t>("support_tree_internal_exponential_threashold");

	volumes_ = ModelVolumes(storage, config.xy_distance,  std::max(config.maximum_move_distance-5,coord_t(0)), std::max(config.maximum_move_distance_slow-5,coord_t(0)), // -5 is to ensure movement is always a bit faster than the avoidance
			radius_sample_resolution, z_distance_bottom_layers, z_distance_top_layers, config.support_rests_on_model,avoid_support_blocker,
			use_legacy_tree_support, use_exponential_collission_resolution, exponential_threashold,exponential_factor);
}


void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    const bool global_use_tree_support = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("support_tree_enable");
    const bool use_legacy_tree_support=Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("support_tree_use_legacy");

    if (!(global_use_tree_support ||
          std::any_of(storage.meshes.cbegin(),
                      storage.meshes.cend(),
                      [](const SliceMeshStorage& m) {
                          return m.settings.get<bool>("support_tree_enable");
                      })))
    {
        return;
    }
    auto t_start = std::chrono::high_resolution_clock::now();
    // only precalculates avoidance and collision
    precalculate(storage);
    auto t_precalc = std::chrono::high_resolution_clock::now();

    if(use_legacy_tree_support){
		std::vector<std::unordered_set<Node*>> contact_nodes(storage.support.supportLayers.size()); //Generate empty layers to store the points in.
		for (SliceMeshStorage& mesh : storage.meshes)
		{
			if (mesh.settings.get<bool>("support_tree_enable"))
			{
				generateContactPoints(mesh, contact_nodes);
			}
		}

		//Drop nodes to lower layers.
		dropNodes(contact_nodes);

		//Generate support areas.
		drawCircles(storage, contact_nodes);

		for (auto& layer : contact_nodes)
		{
			for (Node* p_node : layer)
			{
				delete p_node;
			}
			layer.clear();
		}
		contact_nodes.clear();
    }
    else{

    	std::vector<std::map<SupportElement*,Polygons*>> moveBounds(storage.support.supportLayers.size()); // value is the area where support may be placed. As this is calculated in CreateLayerPathing it is saved and reused in drawAreas

        for (SliceMeshStorage &mesh : storage.meshes) {
    		if (mesh.settings.get<bool>("support_tree_enable")) {
    			generateInitalAreas(mesh,moveBounds,storage);
    		}
    	}
        auto t_gen = std::chrono::high_resolution_clock::now();
        //Expand Influnce areas down.
        createLayerPathing(moveBounds);
        auto t_path = std::chrono::high_resolution_clock::now();
    	//Set a point in each influence area
        createNodesFromArea(moveBounds);
        auto t_place = std::chrono::high_resolution_clock::now();
        //draw these points as circles
    	drawAreas(moveBounds, storage);
        auto t_draw = std::chrono::high_resolution_clock::now();
        auto durGen = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_gen - t_precalc ).count();
        auto durPath = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_path - t_gen ).count();
        auto durPlace =0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_place - t_path ).count();
        auto durDraw = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_draw - t_place ).count();

    	log("Creating inital influence areas: %.3lf Influence area creation: %.3lf ms Placement of Points in InfluenceAreas: %.3lf ms Drawing result as support %.3lf ms\n",durGen,durPath,durPlace,durDraw);

    	for (auto &layer : moveBounds) {
    		for (auto elem : layer){
    			delete elem.second;
    			delete elem.first->area;
    			delete elem.first;
    		}
    	}
    	moveBounds.clear();
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    auto durGen = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_precalc - t_start  ).count();
    auto durTree = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_end - t_precalc ).count();

    log("Precalculation of Avoidance: %.3lf ms Generation of the Tree: %.3lf ms\n",durGen,durTree);

    storage.support.generated = true;
}

coord_t TreeSupport::precalculate(SliceDataStorage &storage){

	size_t maxLayer=0;
	for (SliceMeshStorage &mesh : storage.meshes) {
		const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
		const coord_t z_distance_top = mesh.settings.get<coord_t>(
				"support_top_distance");
		const size_t z_distance_top_layers = round_up_divide(z_distance_top,
				layer_height) + 1; //Support must always be 1 layer below overhang.
		if(mesh.overhang_areas.size()<=z_distance_top_layers)
			continue;

		for (size_t layer_nr = (mesh.overhang_areas.size() - z_distance_top_layers)-1;layer_nr!=0;layer_nr--) { // look for max relevant layer
				const Polygons &overhang = mesh.overhang_areas[layer_nr+ z_distance_top_layers];
				if (!overhang.empty()) {
					if(layer_nr>maxLayer) // iterate over multiple meshes
						maxLayer=1+layer_nr; // plus one to avoid problems if something is of by one
					break;
				}
			}
	}

	volumes_.precalculateAvoidance(maxLayer, config.branch_radius, config.diameter_angle_scale_factor,config.bp_radius,config.diameter_scale_elephant_foot); // legacy code seems to be off by 2
	return maxLayer;
}

void TreeSupport::drawCircles(SliceDataStorage& storage, const std::vector<std::unordered_set<Node*>>& contact_nodes)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    const coord_t branch_radius = mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2;
    const size_t wall_count = mesh_group_settings.get<size_t>("support_wall_count");
    Polygon branch_circle; //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
    for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
    {
        const AngleRadians angle = static_cast<double>(i) / CIRCLE_RESOLUTION * TAU;
        branch_circle.emplace_back(cos(angle) * branch_radius, sin(angle) * branch_radius);
    }
    const coord_t circle_side_length = 2 * branch_radius * sin(M_PI / CIRCLE_RESOLUTION); //Side length of a regular polygon.
    const coord_t z_distance_bottom = mesh_group_settings.get<coord_t>("support_bottom_distance");
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const size_t z_distance_bottom_layers = round_up_divide(z_distance_bottom, layer_height);
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    const double diameter_angle_scale_factor = sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t line_width = mesh_group_settings.get<coord_t>("support_line_width");
    const coord_t resolution = mesh_group_settings.get<coord_t>("support_tree_collision_resolution");
    size_t completed = 0; //To track progress in a multi-threaded environment.
#pragma omp parallel for shared(storage, contact_nodes)
    // Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
    for (int layer_nr = 0; layer_nr < static_cast<int>(contact_nodes.size()); layer_nr++)
    {
        Polygons support_layer;
        Polygons& roof_layer = storage.support.supportLayers[layer_nr].support_roof;

        //Draw the support areas and add the roofs appropriately to the support roof instead of normal areas.
        for (const Node* p_node : contact_nodes[layer_nr])
        {
            const Node& node = *p_node;

            Polygon circle;
            const double scale = static_cast<double>(node.distance_to_top + 1) / tip_layers;
            for (Point corner : branch_circle)
            {
                if (node.distance_to_top < tip_layers) //We're in the tip.
                {
                    const int mul = node.skin_direction ? 1 : -1;
                    corner = Point(corner.X * (0.5 + scale / 2) + mul * corner.Y * (0.5 - scale / 2),
                                   mul * corner.X * (0.5 - scale / 2) + corner.Y * (0.5 + scale / 2));
                }
                else
                {
                    corner = corner * (1 + static_cast<double>(node.distance_to_top - tip_layers) * diameter_angle_scale_factor);
                }
                circle.add(node.position + corner);
            }
            if (node.support_roof_layers_below >= 0)
            {
                roof_layer.add(circle);
            }
            else
            {
                support_layer.add(circle);
            }
        }
        support_layer = support_layer.unionPolygons();
        roof_layer = roof_layer.unionPolygons();
        support_layer = support_layer.difference(roof_layer);
        const size_t z_collision_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(z_distance_bottom_layers) + 1)); //Layer to test against to create a Z-distance.
        support_layer = support_layer.difference(volumes_.getCollision(0, z_collision_layer)); //Subtract the model itself (sample 0 is with 0 diameter but proper X/Y offset).
        roof_layer = roof_layer.difference(volumes_.getCollision(0, z_collision_layer));
        //We smooth this support as much as possible without altering single circles. So we remove any line less than the side length of those circles.
        const double diameter_angle_scale_factor_this_layer = static_cast<double>(storage.support.supportLayers.size() - layer_nr - tip_layers) * diameter_angle_scale_factor; //Maximum scale factor.
        support_layer.simplify(circle_side_length * (1 + diameter_angle_scale_factor_this_layer), resolution); //Don't deviate more than the collision resolution so that the lines still stack properly.

        //Subtract support floors.
        if (mesh_group_settings.get<bool>("support_bottom_enable"))
        {
            Polygons& floor_layer = storage.support.supportLayers[layer_nr].support_bottom;
            const coord_t support_interface_resolution = mesh_group_settings.get<coord_t>("support_interface_skip_height");
            const size_t support_interface_skip_layers = round_up_divide(support_interface_resolution, layer_height);
            const coord_t support_bottom_height = mesh_group_settings.get<coord_t>("support_bottom_height");
            const size_t support_bottom_height_layers = round_up_divide(support_bottom_height, layer_height);
            for(size_t layers_below = 0; layers_below < support_bottom_height_layers; layers_below += support_interface_skip_layers)
            {
                const size_t sample_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(layers_below) - static_cast<int>(z_distance_bottom_layers)));
                constexpr bool no_support = false;
                constexpr bool no_prime_tower = false;
                floor_layer.add(support_layer.intersection(storage.getLayerOutlines(sample_layer, no_support, no_prime_tower)));
            }
            { //One additional sample at the complete bottom height.
                const size_t sample_layer = static_cast<size_t>(std::max(0, static_cast<int>(layer_nr) - static_cast<int>(support_bottom_height_layers) - static_cast<int>(z_distance_bottom_layers)));
                constexpr bool no_support = false;
                constexpr bool no_prime_tower = false;
                floor_layer.add(support_layer.intersection(storage.getLayerOutlines(sample_layer, no_support, no_prime_tower)));
            }
            floor_layer.unionPolygons();
            support_layer = support_layer.difference(floor_layer.offset(10)); //Subtract the support floor from the normal support.
        }

        for (PolygonsPart outline : support_layer.splitIntoParts(false)) //Convert every part into a PolygonsPart for the support.
        {
            storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(outline, line_width, wall_count);
        }
#pragma omp critical (support_max_layer_nr)
        {
            if (!storage.support.supportLayers[layer_nr].support_infill_parts.empty() || !storage.support.supportLayers[layer_nr].support_roof.empty())
            {
                storage.support.layer_nr_max_filled_layer = std::max(storage.support.layer_nr_max_filled_layer, static_cast<int>(layer_nr));
            }
        }
#pragma omp atomic
        completed++;
#pragma omp critical (progress)
        {
            const double progress_contact_nodes = contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN;
            const double progress_current = completed * PROGRESS_WEIGHT_AREAS;
            const double progress_total = completed * PROGRESS_WEIGHT_AREAS;
            Progress::messageProgress(Progress::Stage::SUPPORT, progress_contact_nodes + progress_current, progress_contact_nodes + progress_total);
        }
    }
}


std::string getPolygonAsString(const Polygons& poly){

	std::string ret ="";
	for(auto path : poly){
		for(Point p:path){
			if(ret!="") ret+=",";
			ret+="("+std::to_string(p.X)+","+std::to_string(p.Y)+")";
		}
	}
	return ret;

}


void TreeSupport::mergeHelper(std::map<SupportElement,Polygons>& reduced_new_layer,
		std::map<SupportElement,AABB>& reduced_new_layer_aabb,
		std::map<SupportElement,Polygons>& input,
		std::map<SupportElement,AABB>& input_aabb,
		const std::unordered_map<SupportElement,Polygons>& main,
		const std::unordered_map<SupportElement,Polygons>& secondary,
		std::unordered_map<SupportElement,Polygons>& insertMain,
		std::unordered_map<SupportElement,Polygons>& insertSecondary,
		std::vector<SupportElement>& erase,
		coord_t layer_nr )const{

	for(std::map<SupportElement,Polygons>::iterator inflIt = input.begin(); inflIt != input.end(); inflIt++){
			bool merged=false;

			AABB inflAABB = input_aabb.count(inflIt->first)?input_aabb.at(inflIt->first):AABB(inflIt->second);
			for(std::map<SupportElement,AABB>::iterator redcheck = reduced_new_layer_aabb.begin(); redcheck != reduced_new_layer_aabb.end(); redcheck++){

				AABB aabb = redcheck->second;
				if(aabb.hit(inflAABB)) {

					bool merging_gracious_and_non_gracious = redcheck->first.to_model_gracious != inflIt->first.to_model_gracious; // we do not want to merge a gracious with a non gracious area as bad placement could kill the hole subtree
					bool merging_to_bp=redcheck->first.to_buildplate&&inflIt->first.to_buildplate; // get from main otherwise get from secondary

					size_t increasedDDT=0;
					size_t smaller_to_model_ddt=0;

					if(!merging_to_bp){

						coord_t effDdtInfl=config.getEffektiveDDT(inflIt->first);
						coord_t effDdtRed=config.getEffektiveDDT(redcheck->first);
						if(redcheck->first.to_buildplate!=inflIt->first.to_buildplate){ // calculate increased DDT and total_to_model_ddt
							if(redcheck->first.to_buildplate){
								if(effDdtInfl<effDdtRed){
									increasedDDT=inflIt->first.increased_ddt + redcheck->first.distance_to_top-inflIt->first.distance_to_top;
								}
							}
							else{
								if(effDdtInfl>effDdtRed){
									increasedDDT=redcheck->first.increased_ddt + inflIt->first.distance_to_top-redcheck->first.distance_to_top;
								}
							}
						}
						smaller_to_model_ddt=effDdtInfl<effDdtRed ? effDdtInfl:effDdtRed;

					}

					// if we could place ourself on unstable ground, would be increasing our radius further than we are allowed to when merging to model and to bp trees or we merge 2 non gracious before we know we will even draw this subtree we dont merge
					// also we check if one of both has moved, if no we wont have anything to merge
					if(merging_gracious_and_non_gracious||increasedDDT>config.max_ddt_increase||(smaller_to_model_ddt<config.min_ddt_to_model&& !redcheck->first.to_model_gracious && !inflIt->first.to_model_gracious)){
						continue;
					}

					Polygons relevantInf;
					Polygons relevantRed;

					if(merging_to_bp)
					{
						relevantInf= main.count(inflIt->first)?main.at(inflIt->first):Polygons(); // is a new element dont need to check if i changed it, cause i didnt
						relevantRed= insertMain.count(redcheck->first)?insertMain[redcheck->first]:(main.count(redcheck->first)?main.at(redcheck->first):Polygons());
					}
					else{
						relevantInf= secondary.count(inflIt->first)?secondary.at(inflIt->first):Polygons();
						relevantRed= insertSecondary.count(inflIt->first)?insertSecondary[redcheck->first]:(secondary.count(redcheck->first)?secondary.at(redcheck->first):Polygons());
					}

					const bool redBigger=config.getEffektiveDDT(redcheck->first)>config.getEffektiveDDT(inflIt->first);
					std::pair<SupportElement,Polygons> smallerRad = redBigger ? std::pair<SupportElement,Polygons>(inflIt->first,relevantInf) : std::pair<SupportElement,Polygons>(redcheck->first,relevantRed);
					std::pair<SupportElement,Polygons>biggerRad =redBigger ? std::pair<SupportElement,Polygons>(redcheck->first,relevantRed) : std::pair<SupportElement,Polygons>(inflIt->first,relevantInf);
					const coord_t bigRadius=config.getRadius(biggerRad.first);
					const coord_t smallRadius=config.getRadius(smallerRad.first);
					Polygons smallRadIncreasedByBigMinusSmall;
					Polygons intersect;
					if(config.getCollisionRadius(biggerRad.first)<config.getCollisionRadius(smallerRad.first)){
						smallRadIncreasedByBigMinusSmall=biggerRad.second.offset(bigRadius-smallRadius, ClipperLib::jtRound);
						intersect= smallRadIncreasedByBigMinusSmall.intersection(smallerRad.second);
					}
					else{
						smallRadIncreasedByBigMinusSmall=smallerRad.second.offset(bigRadius-smallRadius, ClipperLib::jtRound);
						intersect= smallRadIncreasedByBigMinusSmall.intersection(biggerRad.second);
					}

					if(intersect.area()>1){ // dont use empty as a line is not empty but for our usecase it very well may be (and will be one layer down, union does not keep lines)

						if(intersect.offset(-25).area()<=1) // check if the overlap we have is good enough. While 25 was guessed as enough, until now i did not have reason to doubt that
							continue;

						Point newPos = redcheck->first.next_position;
						if(!intersect.inside(newPos, true))
							PolygonUtils::moveInside(intersect, newPos);

						if(increasedDDT==0)
							increasedDDT=std::max(redcheck->first.increased_ddt,inflIt->first.increased_ddt);

						SupportElement key(redcheck->first,inflIt->first,layer_nr-1,newPos,increasedDDT,config);

						Polygons intersectSec;
						if(merging_to_bp){

							Polygons secSmall=insertSecondary.count(smallerRad.first)?insertSecondary[smallerRad.first]:(secondary.count(smallerRad.first)?secondary.at(smallerRad.first):Polygons());
							Polygons secBig=insertSecondary.count(biggerRad.first)?insertSecondary[biggerRad.first]:(secondary.count(biggerRad.first)?secondary.at(biggerRad.first):Polygons());
							Polygons smallRadIncreasedByBigMinusSmallSec;
							if(config.getCollisionRadius(biggerRad.first)<config.getCollisionRadius(smallerRad.first)){
								smallRadIncreasedByBigMinusSmallSec=secBig.offset(bigRadius-smallRadius, ClipperLib::jtRound);
								intersectSec= smallRadIncreasedByBigMinusSmallSec.intersection(secSmall);// if the one with the bigger radius with the lower radius removed overlaps we can merge
							}
							else{

								smallRadIncreasedByBigMinusSmallSec=secSmall.offset(bigRadius-smallRadius, ClipperLib::jtRound);
								intersectSec= smallRadIncreasedByBigMinusSmallSec.intersection(secBig);// if the one with the bigger radius with the lower radius removed overlaps we can merge
							}

						}

						insertMain.erase(redcheck->first);
						insertMain.erase(inflIt->first);
						insertSecondary.erase(redcheck->first);
						insertSecondary.erase(inflIt->first);

						(merging_to_bp?insertMain:insertSecondary).emplace(key,intersect);
						if(merging_to_bp)
							insertSecondary.emplace(key,intersectSec);
						erase.emplace_back(redcheck->first);
						erase.emplace_back(inflIt->first);
						Polygons merge = intersect.unionPolygons(intersectSec).offset(config.getRadius(key), ClipperLib::jtRound).difference(volumes_.getCollision(0, layer_nr-1)); //we dont need to safe offset here as it should already be correctly of the intersect area being away enough

						reduced_new_layer.erase(redcheck->first);
						reduced_new_layer.emplace(key,merge);

						reduced_new_layer_aabb.erase(redcheck->first); // this invalidates redcheck
						reduced_new_layer_aabb.emplace(key,AABB(merge));

						merged=true;
						break;
					}
				}
			}

			if(!merged){
				reduced_new_layer[inflIt->first]=inflIt->second;
				reduced_new_layer_aabb[inflIt->first]=inflAABB;
			}

		}
}


void TreeSupport::mergePolygonsDaQ(std::map<SupportElement,Polygons>& input,std::unordered_map<SupportElement,Polygons>& main,std::unordered_map<SupportElement,Polygons>& secondary,int layer_nr)const{
	const size_t input_size =input.size();
	size_t num_threads =1;
#if defined(_OPENMP)
	num_threads =omp_get_max_threads();
#endif
	if(input_size==0)
		return;
	constexpr int min_elements_per_bucket=2;
	const size_t buckets =0.1+std::pow(2, std::ceil(std::log(round_up_divide(input_size,min_elements_per_bucket))/std::log(2))); // todo something pretty against rounding errors
	int num_parts=std::min(buckets,num_threads);

	std::vector<std::map<SupportElement,Polygons>> parts(2*num_parts);
	std::vector<std::map<SupportElement,AABB>> parts_aabb(2*num_parts);
	size_t position=0,counter=0;
	const size_t over_elements= input_size%num_parts;
	const size_t elements_per_step=input_size/num_parts;
	// we split the data in x parts to be able to divide and conqer
	for(std::map<SupportElement,Polygons>::iterator iter = input.begin(); iter != input.end(); iter++){
		parts[position*2+1].emplace(iter->first,iter->second);// only use every second bucket beginning with 1 as this makes the parallel call later easier as we assume everything in a bucker i%2==0 is already processed
		counter++;
		if((counter==elements_per_step&&position>=over_elements)||counter>elements_per_step){
			position++;
			counter=0;
		}

	}

	while(parts.size()>1){

		std::vector<std::unordered_map<SupportElement,Polygons>> insertMain(parts.size()/2);
		std::vector<std::unordered_map<SupportElement,Polygons>> insertSecondary(parts.size()/2);
		std::vector<std::vector<SupportElement>> erase(parts.size()/2);

		#pragma omp parallel for schedule(dynamic)
		for(coord_t i=0;i<(coord_t)parts.size()-1;i=i+2){

			mergeHelper(parts[i], parts_aabb[i], parts[i+1],parts_aabb[i+1], main, secondary, insertMain[i/2], insertSecondary[i/2], erase[i/2], layer_nr);
			parts[i+1].clear();
			parts_aabb[i+1].clear();

		}

		for(coord_t i=0;i<(coord_t)parts.size()-1;i=i+2){

			for(SupportElement& del : erase[i/2]){
				main.erase(del);
				secondary.erase(del);
			}

			for(const std::pair<SupportElement,Polygons>& tup : insertMain[i/2]){
				main.emplace(tup);
			}

			for(const std::pair<SupportElement,Polygons>& tup : insertSecondary[i/2]){
				secondary.emplace(tup);
			}
		}

		auto position = std::remove_if(parts.begin(), parts.end(),
			[&](const std::map<SupportElement,Polygons> x) mutable {return x.empty();});
		parts.erase(position, parts.end());

		auto position_aabb = std::remove_if(parts_aabb.begin(), parts_aabb.end(),
			[&](const std::map<SupportElement,AABB> x) mutable {return x.empty();});
		parts_aabb.erase(position_aabb, parts_aabb.end());

	}

	input.clear(); // //insert parts back to input
	input.insert(parts[0].begin(),parts[0].end());


}

Polygons TreeSupport::safeOffsetInc(Polygons& me,coord_t distance,const Polygons& collision,coord_t lastSafeStepSize,size_t min_amount_offset)const{

	if(distance==0)
		return me.difference(collision).unionPolygons();

	size_t counted=0;
	size_t steps = distance>lastSafeStepSize ? std::abs ((distance-lastSafeStepSize)/config.xy_distance):0;
	size_t step_size=config.xy_distance;
	if(steps+(distance<lastSafeStepSize||distance%step_size!=0)<min_amount_offset ){ // yes we can add a bool as the standard specifies that a resolt from comare operators has to be 0 or 1
		// we need to reduce the stepsize to ensure we offset the required amount (could be avoided if arcRadiance were exposed as we could just increase this when me has not enough vertecies)
		step_size=distance/(min_amount_offset-1);
		steps= distance/step_size;
	}

	Polygons ret=me;

	for(size_t i=0;i<steps;i++){ // offset the steps
		ret=ret.offset(step_size,ClipperLib::jtRound).difference(collision);
		counted+= step_size;
	}

	ret= ret.offset(distance-steps*step_size,ClipperLib::jtRound); // offset the remainder

	return ret.difference(collision).unionPolygons(); // ensure sane output
}

//increases all polygons of the last layer and adds them to newLayer newLayer_to_model and newLayer_merge
void TreeSupport::increaseAreas(std::unordered_map<SupportElement,Polygons>& newLayer,std::map<SupportElement,Polygons>& newLayer_merge,std::unordered_map<SupportElement,Polygons>& newLayer_to_model,std::vector<std::pair<SupportElement*,Polygons*>> lstLayer,size_t layer_nr){

#pragma omp parallel for shared(newLayer,newLayer_merge,newLayer_to_model,lstLayer)
		for(long long unsigned int i=0;i<lstLayer.size();i++){

			std::pair<SupportElement*,Polygons*> tup = lstLayer[i];

			SupportElement elem(*tup.first,tup.first); // also increases ddt

			coord_t extraOverspeed=0;

			// we assume that the good areas from our new collision will overlap with the good areas of our old collision, but we have to use 0 for our old collision as we may not fit a whole circle in there in some cases i think
			// this may not be the most efficient way as a bigger collision would increase our safeStepSize, but it is a correct one
			const Polygons wallRestriction=volumes_.getCollision(0, layer_nr).intersection(volumes_.getCollision(config.getRadius(elem.effective_radius_height), layer_nr-1)); // second also 0 rad ?

			//Polygons newLayerDataSlow=tup.first->area->safeOffset(config.maximum_move_distance_slow,ClipperLib::jtRound,max_step_move,wallRestriction,true); // we need to offset in 2 steps as we loose areas if we only inc small areas once sometimes
			Polygons newLayerData,newLayerDataToModel,checkLayerData,increased;
			size_t radius;
			std::function<bool(bool,bool,bool)> setPolygons=[&](bool slow,bool increaseRadius,bool simplify) { // this correctly sets the polygons defined above according to the parameters and returns if a new layer is possible
				SupportElement simulateIncrease(elem);
				if(increaseRadius)
					simulateIncrease.effective_radius_height+=1;
				radius = increaseRadius? config.getCollisionRadius(simulateIncrease) : config.getCollisionRadius(elem);

				coord_t extraMoveSpeed= config.getRadius(simulateIncrease)-config.getRadius(*tup.first); //todo limit ?
				if(extraMoveSpeed<0)
					logWarning("Tree Support: extraMoveSpeed<0. This implies that a branch is getting smaller while going down. This may cause problems.");
				extraMoveSpeed+=extraOverspeed;
				increased=safeOffsetInc(*tup.first->area, extraMoveSpeed + (slow?config.maximum_move_distance_slow:config.maximum_move_distance), wallRestriction, 2*(config.xy_distance+radius), 2); // offsetting in 2 steps makes our offseted area rounder preventing (rounding) errors created by to pointy areas. May not be a problem anymore though.

				if(simplify){
					increased=increased.smooth(5);
					increased.simplify(15);
				}
				newLayerData= increased.difference(volumes_.getAvoidance(radius,layer_nr-1,slow)).unionPolygons();
				if(config.support_rests_on_model){
					 newLayerDataToModel= increased.difference(volumes_.getAvoidance(radius, layer_nr-1,slow , true)).unionPolygons();
					 if(!elem.to_model_gracious){
						 if(newLayerDataToModel.area()>=1){
							 elem.to_model_gracious=true;
							 logWarning("Corrected to model taint on layer %lld targeting %lld with radius %lld",layer_nr-1,elem.target_height,radius);

						 }
						 else{
							 newLayerDataToModel= increased.difference(volumes_.getCollision(radius, layer_nr-1)).unionPolygons();
						 }
					 }
				}
				if(newLayerData.area()>1&& !elem.to_buildplate){ //mostly happening in the tip, but with merges ...
					elem.to_buildplate=true; // sometimes nodes that can reach the buildplate are marked as cant reach, taining subtrees. this fixes this
					logWarning("Corrected to buildplate taint on layer %lld targeting %lld with radius %lld",layer_nr-1,elem.target_height,radius);
				}
				checkLayerData=elem.to_buildplate ? newLayerData : newLayerDataToModel;
				if(increaseRadius&&checkLayerData.area()>1){
					std::function<bool(bool,bool,bool)> nextStep=[&](bool slow,bool increaseRadius,bool simplify) { // saves current status, and trys another step with the same settings to check if the change was valid
						Polygons newLayerData_backup=Polygons(newLayerData); // remember, setPolygons is by DEFINITION not non destructive
						Polygons checkLayerData_backup=Polygons(checkLayerData);
						Polygons newLayerDataToModel_backup=Polygons(newLayerDataToModel);

						if(!setPolygons(slow,increaseRadius,simplify)){ // we can not catch up further:(
							checkLayerData=checkLayerData_backup;
							newLayerData=newLayerData_backup;
							newLayerDataToModel=newLayerDataToModel_backup;
							return false;
						}
						return true;
					};
					elem.effective_radius_height+=1;
					bool increase_bp_foot =config.reccomendedMinRadius(layer_nr-1)>config.getRadius(elem)&&elem.to_buildplate;

					if(increase_bp_foot&&config.getRadius(elem)>=config.branch_radius){ // we add a 45 deg line check and call it a day
						elem.elephant_foot_increases+=1;
						if(!nextStep(slow,false,simplify)){
							elem.elephant_foot_increases-=1;
						}
					}

					if(config.getRadius(elem) < config.increase_radius_until_radius&&elem.effective_radius_height<elem.distance_to_top){

						coord_t orig_ceil= volumes_.ceilRadius(config.getRadius(elem.effective_radius_height));
						bool increasingRad=config.getRadius(elem.effective_radius_height)!=config.getRadius(elem.effective_radius_height+1);
						size_t maxEff = elem.effective_radius_height;
						for (coord_t i =1;volumes_.ceilRadius(config.getRadius(elem.effective_radius_height+i))==orig_ceil&&increasingRad;i++){ // todo make it mathematically nice, instead of simulating radius increases

							if(config.getRadius(elem.effective_radius_height+i)==config.getRadius(elem.effective_radius_height+i+1))// we reached a point where we can not increase our radius further
							{
								maxEff=std::numeric_limits<coord_t>::max(); // yeah i know this is missing one bit, i just want to make sure i dont accidently overlow
								break;
							}
							maxEff++;
						}
						elem.effective_radius_height=maxEff>config.getEffektiveDDT(elem)? config.getEffektiveDDT(elem):maxEff;
						bool another_step =maxEff<config.getEffektiveDDT(elem);

						if(another_step){
							nextStep(slow,true,simplify);
						}
					}
				}

				return checkLayerData.area()>1;
			};


			bool add=false;

			if(		setPolygons(true,true,true)||
					setPolygons(true,false,true)|| // todo proper ordering Do we want to move FAST always when we go to model ? Do i want to move fast to increase bp_radius ?
					setPolygons(false,true,true)||
					setPolygons(false,false,true)){

				add=true;
			}
			else{

				if(!elem.to_buildplate&&(!elem.to_model_gracious ||( !tup.first->area->intersection(volumes_.getPlaceableAreas(radius, layer_nr)).empty()))) // we can use empty here as we dont need area inside the polygon i think
					continue; // it is normal that we wont be able to find a new area at some point in time if we wont be able to reach layer 0 aka have to connect with the model

				size_t old_radius=radius;
				SupportElement old_elem=elem;

				extraOverspeed=config.maximum_move_distance/2;
				if(setPolygons(false,false,false)){
					logError("Influence area could not be increased! Data about the Influence area: Radius: %lld at layer: %lld NextTarget: %lld Distance to top: %lld Trying to keep area by moving faster than intended: Success \n",old_radius,layer_nr-1,old_elem.next_height,old_elem.distance_to_top);
					add=true;
				}
				else{
					logError("Influence area could not be increased! Data about the Influence area: Radius: %lld at layer: %lld NextTarget: %lld Distance to top: %lld Trying to keep area by moving faster than intended: FAILURE! WRONG BRANCHES LIKLY! \n",old_radius,layer_nr-1,old_elem.next_height,old_elem.distance_to_top);
				}
			}

			if(add){

				Polygons merge; // we should make sure that the outer wall is correct

				Polygons mergeInc=(config.support_rests_on_model?newLayerDataToModel:newLayerData).offset(config.getRadius(elem), ClipperLib::jtRound);
				merge = mergeInc.difference(volumes_.getCollision(0, layer_nr-1));

				#pragma omp critical(newLayer)
				{
					newLayer_merge.emplace(elem,merge);
					if(elem.to_buildplate)
						newLayer.emplace(elem,newLayerData);
					if(config.support_rests_on_model)
						newLayer_to_model.emplace(elem,newLayerDataToModel);
				}
			}

		}

}


void TreeSupport::createLayerPathing(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds){

	const double data_size_inverse = 1/double(moveBounds.size());
	double progress_total =PROGRESS_PRECALC_AVO+PROGRESS_PRECALC_COLL+PROGRESS_GENERATE_NODES;

	auto durInc=std::chrono::duration_values<std::chrono::nanoseconds>::zero();
	auto durMerge=std::chrono::duration_values<std::chrono::nanoseconds>::zero();

	//Increase Influence Radius of each influence circle check for overlap and insert a new influence circle
	for (size_t layer_nr = moveBounds.size() - 1; layer_nr > 0; layer_nr--){

		std::map<SupportElement,Polygons> newLayer_merge; // merge maps are increased by effective radius to be able to ensure correct merge when 2 elements have a different radius
		std::unordered_map<SupportElement,Polygons> newLayer,newLayer_to_model;
		auto ta = std::chrono::high_resolution_clock::now();

		std::vector<std::pair<SupportElement*,Polygons*>> lstLayer;
		lstLayer.insert(lstLayer.begin(), moveBounds[layer_nr].begin(), moveBounds[layer_nr].end());

		increaseAreas(newLayer, newLayer_merge, newLayer_to_model, lstLayer, layer_nr);

		auto tb = std::chrono::high_resolution_clock::now();

		mergePolygonsDaQ(newLayer_merge, newLayer, newLayer_to_model, layer_nr);


		auto tc = std::chrono::high_resolution_clock::now();

		durInc+=tb-ta;
		durMerge+=tc-tb;


		for(std::pair<SupportElement,Polygons> tup:newLayer){
			const SupportElement elem = tup.first;
			Polygons* new_area= new Polygons(tup.second.unionPolygons(newLayer_to_model.count(elem)?newLayer_to_model[elem]:Polygons()));
			SupportElement* next= new SupportElement(elem,new_area);
			moveBounds[layer_nr-1][next]=new Polygons(newLayer_merge[tup.first]);

			if(new_area->area()<1){
				logError("Insert Error of Influence area to buildplate on layer %lld.\n",layer_nr-1);
			}
		}

		for(std::pair<SupportElement,Polygons> tup:newLayer_to_model){
			const SupportElement& elem = tup.first;

			if(elem.to_buildplate) continue;
			Polygons* new_area= new Polygons(tup.second);
			SupportElement* next= new SupportElement(elem,new_area);
			moveBounds[layer_nr-1][next]=new Polygons(newLayer_merge[tup.first]);

			if(new_area->area()<1)
				logError("Insert Error of Influence area to model on layer %lld.\n",layer_nr-1);

		}

		progress_total+=data_size_inverse * PROGRESS_AREA_CALC;
		Progress::messageProgress(Progress::Stage::SUPPORT, progress_total,PROGRESS_TOTAL);

	}

    log("Total time increasing influence areas: %lld ms Total time merging influence areas: %lld ms\n",durInc.count()/1000000,durMerge.count()/1000000);


}


void TreeSupport::setPointsOnAreas(SupportElement* elem, coord_t max_move){

	if(elem->result_on_layer==Point(-1,-1)){
		printf("ERROR Uninitialised support element \n");
		return;
	}

	for(SupportElement* nextElem : elem->parents){

		if(nextElem->result_on_layer!=Point(-1,-1)) // if we set the value somewhere else we keep it; should only happen at the top most layer
			continue;

		Point from = elem->result_on_layer;
		if(!(nextElem->area->inside(from, true))){ // todo else try move to next position
			PolygonUtils::moveInside(*nextElem->area,from,0); // Move inside has edgecases (see tests) so DONT use Polygons.inside, Error with dist 0 is <= 1 //
		}
		if(vSize(from-elem->result_on_layer)>max_move){ // while this SEEMS like a problem it may occur after merges or because the radius changed => which is free movement speed

		}
		nextElem->result_on_layer=from;
		//no we do not call recursive because if we do out amount of layers is restricted by our stack size -.-'
	}

}

bool TreeSupport::setToModelContact (std::vector<std::map<SupportElement*,Polygons*>>& moveBounds, SupportElement* firstElem ,const size_t layer_nr){


	if(firstElem->to_model_gracious){

		SupportElement* check=firstElem;

		std::vector<SupportElement*> checked({check});
		size_t lastSuccessfullLayer=layer_nr;

		for(size_t layer_check=layer_nr;check->next_height<layer_check;layer_check++){

			if(!check->area->intersection(volumes_.getPlaceableAreas(config.getRadius(*check), layer_check)).empty()){
				lastSuccessfullLayer=layer_check;
			}
			if(layer_check!=layer_nr)
				checked.emplace_back(check);
			if(check->parents.size()==1)
				check=check->parents[0];
			else{
				if(layer_check+1<check->next_height)
					logError("Set To Model Contact encountered a problem: Element that did not merge has multiple parents! \n");
				break;
			}
		}


		for(size_t layer=layer_nr+1;layer<lastSuccessfullLayer-1;layer++){
			//printf("%lld \n",checked[layer-layer_nr]);
			moveBounds[layer].erase(checked[layer-layer_nr]);
			delete checked[layer-layer_nr]->area;
			delete checked[layer-layer_nr];
		}

		// set Pos
		Point best=checked[lastSuccessfullLayer-layer_nr]->next_position;
		if(!checked[lastSuccessfullLayer-layer_nr]->area->inside(best, true))
			PolygonUtils::moveInside(*checked[lastSuccessfullLayer-layer_nr]->area,best);
		checked[lastSuccessfullLayer-layer_nr]->result_on_layer=best;

		logDebug("Added gracious Support On Model on layer Point (%lld,%lld) on current layer is %lld\n",best.X,best.Y,lastSuccessfullLayer);

		return lastSuccessfullLayer!=layer_nr;


	}
	else{ // can not add gracefull => just place it here and hope for the best
		Point best=firstElem->next_position;
		if(!firstElem->area->inside(best, true))
			PolygonUtils::moveInside(*firstElem->area,best);
		firstElem->result_on_layer=best;
		logDebug("added NON gracious  Support On Model on layer Point (%lld,%lld) my current layer is %lld\n",best.X,best.Y,layer_nr);
		return false;
	}

}



void TreeSupport::createNodesFromArea(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds){

	const coord_t maximum_move_distance =config.maximum_move_distance + 3; // movement needs to be a bit higher to avoid rounding errors

	for(std::pair<SupportElement*,Polygons*> init : moveBounds[0]){ // init points on layer 0
		Point p=init.first->next_position;
		if(!(init.first->area->inside(p, true)))
			PolygonUtils::moveInside(*init.first->area,p,0 );
		init.first->result_on_layer=p;
		setPointsOnAreas(init.first, maximum_move_distance);
	}

	std::map<SupportElement,int> ignore;

	for (size_t layer_nr = 1; layer_nr < moveBounds.size(); layer_nr++){
		std::unordered_set<SupportElement*> remove;
		for(std::pair<SupportElement*,Polygons*> nodeData : moveBounds[layer_nr]){

			bool removed=false;
			if(nodeData.first->result_on_layer==Point(-1,-1)){
				if(nodeData.first->to_buildplate){
					logError("UNKNOWN POLYGON targeting (%lld,%lld) at target_height: %lld layer: %lld\n",nodeData.first->target_position.X,nodeData.first->target_position.Y,nodeData.first->target_height,layer_nr);
				}
				else{// we need to connect with the model

					if(config.getEffektiveDDT(*nodeData.first)<config.min_ddt_to_model){
						remove.emplace(nodeData.first); // we dont need to remove the parrents as they will have a lower ddt
						removed=true;
						for(SupportElement* elem:nodeData.first->parents)
							elem->to_buildplate=false;
						continue;
					}

					removed=setToModelContact(moveBounds, nodeData.first, layer_nr);
					if(removed)
						remove.emplace(nodeData.first);
				}
			}

			if(!removed)
				setPointsOnAreas(nodeData.first, maximum_move_distance);//element is valid now setting points in the layer above
		}

		for(SupportElement* del :remove){ // delete all support elements we dont need anymore
			delete moveBounds[layer_nr][del];
			moveBounds[layer_nr].erase(del);
			delete del->area;
			delete del;
		}
		remove.clear();
	}

}

void TreeSupport::drawAreas(std::vector<std::map<SupportElement*,Polygons*>>& moveBounds,SliceDataStorage &storage){
	//we do most of the things as in the original draw circle but we can improve a few things as we already calculated maximal outer bounds for our tree wall we can now cut areas out of them elliminating the need for a slow difference, using a much faster intersect instead
	//also we can add the z- bottom distance to precalc

	const Settings &mesh_group_settings =
				Application::getInstance().current_slice->scene.current_mesh_group->settings;
		const coord_t branch_radius = mesh_group_settings.get<coord_t>(
				"support_tree_branch_diameter") / 2;
		const size_t wall_count = mesh_group_settings.get<size_t>(
				"support_wall_count");
		Polygon branch_circle; //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
		for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++) {
			const AngleRadians angle = static_cast<double>(i) / CIRCLE_RESOLUTION
					* TAU;
			branch_circle.emplace_back(cos(angle) * branch_radius,
					sin(angle) * branch_radius);
		}
		const coord_t z_distance_bottom = mesh_group_settings.get<coord_t>(
				"support_bottom_distance");
		const coord_t layer_height = mesh_group_settings.get<coord_t>(
				"layer_height");
		const size_t z_distance_bottom_layers = round_up_divide(z_distance_bottom,
				layer_height);
		const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
		const double diameter_angle_scale_factor = sin(
				mesh_group_settings.get<AngleRadians>(
						"support_tree_branch_diameter_angle")) * layer_height
				/ branch_radius; //Scale factor per layer to produce the desired angle.
		const coord_t line_width = mesh_group_settings.get<coord_t>(
				"support_line_width");
		const std::function<SupportElement*(SupportElement*)> getBiggestRadiusParrent=[&](const SupportElement* elem) {
			SupportElement* ret=nullptr;
			for(SupportElement* el2:elem->parents){
				const coord_t rad=config.getRadius(*el2);
				if(ret==nullptr||rad>config.getRadius(*ret))
					ret=el2;
			}
			return  ret;
		};

	std::map<SupportElement*,std::pair<SupportElement*,Polygons*>> invereseTreeOrder; // in our tree we can only access the partents. We inverse this to be able to access the children.
	std::vector<std::pair<size_t,std::pair<SupportElement*,Polygons*>>> linearData; // we put all SupportElements into a layer independent storage to improve parallelization. Was added at a point in time where this function had performance issues.
																					// These were fixed by creating less initial points, but i do not see a good reason to remove a working performance optimization.
	for (size_t layer_nr = 0; layer_nr < moveBounds.size();layer_nr++) {
		for(std::pair<SupportElement*,Polygons*> tup: moveBounds[layer_nr]){

			if((layer_nr>0&&((!invereseTreeOrder.count(tup.first)&&tup.first->target_height==layer_nr)||(invereseTreeOrder.count(tup.first)&&invereseTreeOrder[tup.first].first->result_on_layer==Point(-1,-1))))) // we either come from nowhere at the final layer or we had invalid parents 2. should never happen but just to be sure
				continue;

			for(SupportElement* par : tup.first->parents){
				if(par->result_on_layer==Point(-1,-1))
					continue;
				invereseTreeOrder.emplace(par,tup);
			}
			linearData.emplace_back(layer_nr,tup);
		}
	}

	std::vector<Polygons> linearInserts(linearData.size());
	// parallel iterating over all elements
#pragma omp parallel for shared(linearInserts, linearData,invereseTreeOrder)
	for(coord_t i=0;i<static_cast<coord_t>(linearData.size());i++){
		const std::pair<SupportElement*,Polygons*> tup= linearData[i].second;

		Polygon circle;
		double scale = (1.0 + static_cast<double>(config.getEffektiveDDT(*tup.first) - tip_layers) * diameter_angle_scale_factor);
		if (config.getEffektiveDDT(*tup.first) < tip_layers){ //We're in the tip.
			scale = static_cast<double>(config.getEffektiveDDT(*tup.first) + 1) / tip_layers;
		}
		scale = config.getRadius(*tup.first)/(1.0*branch_radius);

		Point movement;
		SupportElement* childElem =  invereseTreeOrder.count(tup.first)?invereseTreeOrder.at(tup.first).first:nullptr;
		if(childElem!=nullptr){
			movement =(childElem->result_on_layer - tup.first->result_on_layer);
			if(tup.first->next_height==linearData[i].first&&movement.X+movement.Y){ // if merge limit move distance to avoid excessive ovalization
				coord_t radiusDifference=  std::abs(config.getRadius(*tup.first)-config.getRadius(*childElem));
				movement=movement*(vSize(movement)-radiusDifference>0?vSize(movement)-radiusDifference:0)/vSize(movement);
			}
		}
		else if(tup.first->parents.size()!=0){//only happens at the most bottom area of the support
			SupportElement* biggestRadiusParrent=getBiggestRadiusParrent(tup.first);
			movement =( biggestRadiusParrent->result_on_layer - tup.first->result_on_layer);
			if(biggestRadiusParrent->next_position==linearData[i].first&&movement.X+movement.Y){ // if merge limit move distance to avoid excessive ovalization
				coord_t radiusDifference=  std::abs(config.getRadius(*tup.first)-config.getRadius(*biggestRadiusParrent));
				movement=movement*(vSize(movement)-radiusDifference>0?vSize(movement)-radiusDifference:0)/vSize(movement);
			}
		}
		else{
			logWarning("Ovalisation failed! This means a element has neither a child nor a parent!\n");
		}

		Point centerPosition = tup.first->result_on_layer+movement/2;

		const double moveX = movement.X/(scale*branch_radius);
		const double moveY = movement.Y/(scale*branch_radius);
		const double vsize_inv=0.5/(0.01+std::sqrt(moveX*moveX+moveY*moveY));

		double matrix[] = {
			scale* (1 + moveX*moveX*vsize_inv),
			scale* (0 + moveX*moveY*vsize_inv),
			scale* (0 + moveX*moveY*vsize_inv),
			scale* (1 + moveY*moveY*vsize_inv),
		};

		for (Point vertex : branch_circle) {
			vertex = Point(
					matrix[0]*vertex.X + matrix[1]*vertex.Y,
					matrix[2]*vertex.X + matrix[3]*vertex.Y);
			circle.add(centerPosition + vertex);
		}


		Polygons poly;
		poly.add(circle); // ensures the point always exists

		const Polygons child = invereseTreeOrder.count(tup.first)?*invereseTreeOrder.at(tup.first).second:Polygons();
		linearInserts[i]=tup.second->unionPolygons(child).intersection(poly.unionPolygons());
	}

	std::vector<std::vector<Polygons>> byLayer(moveBounds.size());
	// single threaded combinating all elements to the right layers ONLY COPYS DATA
	for(coord_t i=0;i<static_cast<coord_t>(linearData.size());i++){
		if (config.support_roof_layers>=linearData[i].second.first->distance_to_top){
			storage.support.supportLayers[linearData[i].first].support_roof.add(linearInserts[i]);
		}
		else{
			precalculatedSupportLayers[linearData[i].first].add(linearInserts[i]);
		}
	}


	linearInserts.clear();
	linearData.clear();

	//now we iterate over the inserted elements in parallel and clean them up
#pragma omp parallel for shared(precalculatedSupportLayers, storage)
	for (coord_t layer_nr = 0; layer_nr < static_cast<coord_t>(precalculatedSupportLayers.size());layer_nr++) {
			precalculatedSupportLayers[layer_nr] = precalculatedSupportLayers[layer_nr].unionPolygons().smooth(50);
			storage.support.supportLayers[layer_nr].support_roof = storage.support.supportLayers[layer_nr].support_roof.unionPolygons();
			precalculatedSupportLayers[layer_nr] = precalculatedSupportLayers[layer_nr].difference(storage.support.supportLayers[layer_nr].support_roof);
			precalculatedSupportLayers[layer_nr].simplify(30,10);
			//Subtract support floors.
			if (mesh_group_settings.get<bool>("support_bottom_enable")) { // support bottom magic from the drawCircle.
				Polygons &floor_layer =
						storage.support.supportLayers[layer_nr].support_bottom;
				const coord_t support_interface_resolution =
						mesh_group_settings.get<coord_t>(
								"support_interface_skip_height");
				const size_t support_interface_skip_layers = round_up_divide(
						support_interface_resolution, layer_height);
				const coord_t support_bottom_height = mesh_group_settings.get<
						coord_t>("support_bottom_height");
				const size_t support_bottom_height_layers = round_up_divide(
						support_bottom_height, layer_height);
				for (size_t layers_below = 0;
						layers_below < support_bottom_height_layers; layers_below +=
								support_interface_skip_layers) {
					const size_t sample_layer = static_cast<size_t>(std::max(0,
							static_cast<int>(layer_nr)
									- static_cast<int>(layers_below)
									- static_cast<int>(z_distance_bottom_layers)));
					constexpr bool no_support = false;
					constexpr bool no_prime_tower = false;
					floor_layer.add(
							precalculatedSupportLayers[layer_nr].intersection(
									storage.getLayerOutlines(sample_layer,
											no_support, no_prime_tower)));
				}
				{ //One additional sample at the complete bottom height.
					const size_t sample_layer = static_cast<size_t>(std::max(0,
							static_cast<int>(layer_nr)
									- static_cast<int>(support_bottom_height_layers)
									- static_cast<int>(z_distance_bottom_layers)));
					constexpr bool no_support = false;
					constexpr bool no_prime_tower = false;
					floor_layer.add(
							precalculatedSupportLayers[layer_nr].intersection(
									storage.getLayerOutlines(sample_layer,
											no_support, no_prime_tower)));
				}
				floor_layer.unionPolygons();
				precalculatedSupportLayers[layer_nr] = precalculatedSupportLayers[layer_nr].difference(floor_layer.offset(10)); //Subtract the support floor from the normal support.
			}

		for (PolygonsPart part : precalculatedSupportLayers[layer_nr].splitIntoParts(false) ) //Convert every part into a PolygonsPart for the support.
		{
			PolygonsPart outline;
			outline.add(part);
			storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(
					outline, line_width, wall_count);
		}

	#pragma omp critical (support_max_layer_nr)
			{
				if (!storage.support.supportLayers[layer_nr].support_infill_parts.empty()
						|| !storage.support.supportLayers[layer_nr].support_roof.empty()) {
					storage.support.layer_nr_max_filled_layer = std::max(
							storage.support.layer_nr_max_filled_layer,
							static_cast<int>(layer_nr));
				}
			}

		}


}

void TreeSupport::dropNodes(std::vector<std::unordered_set<Node*>>& contact_nodes)
{
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    //Use Minimum Spanning Tree to connect the points on each layer and move them while dropping them down.
    const coord_t layer_height = mesh_group_settings.get<coord_t>("layer_height");
    const double angle = mesh_group_settings.get<AngleRadians>("support_tree_angle");
    const coord_t maximum_move_distance = angle < 90 ? static_cast<coord_t>(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max();
    const coord_t branch_radius = mesh_group_settings.get<coord_t>("support_tree_branch_diameter") / 2;
    const size_t tip_layers = branch_radius / layer_height; //The number of layers to be shrinking the circle to create a tip. This produces a 45 degree angle.
    const double diameter_angle_scale_factor = sin(mesh_group_settings.get<AngleRadians>("support_tree_branch_diameter_angle")) * layer_height / branch_radius; //Scale factor per layer to produce the desired angle.
    const coord_t radius_sample_resolution = mesh_group_settings.get<coord_t>("support_tree_collision_resolution");
    const bool support_rests_on_model = mesh_group_settings.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;

    std::unordered_set<Node*> to_free_node_set;

    for (size_t layer_nr = contact_nodes.size() - 1; layer_nr > 0; layer_nr--) //Skip layer 0, since we can't drop down the vertices there.
    {
        auto& layer_contact_nodes = contact_nodes[layer_nr];
        std::deque<std::pair<size_t, Node*>> unsupported_branch_leaves; // All nodes that are leaves on this layer that would result in unsupported ('mid-air') branches.

        //Group together all nodes for each part.
        std::vector<PolygonsPart> parts = volumes_.getAvoidance(0, layer_nr).splitIntoParts();
        std::vector<std::unordered_map<Point, Node*>> nodes_per_part;
        nodes_per_part.emplace_back(); //All nodes that aren't inside a part get grouped together in the 0th part.
        for (size_t part_index = 0; part_index < parts.size(); part_index++)
        {
            nodes_per_part.emplace_back();
        }
        for (Node* p_node : layer_contact_nodes)
        {
            const Node& node = *p_node;

            if (!support_rests_on_model && !node.to_buildplate) //Can't rest on model and unable to reach the build plate. Then we must drop the node and leave parts unsupported.
            {
                unsupported_branch_leaves.push_front({ layer_nr, p_node });
                continue;
            }
            if (node.to_buildplate || parts.empty()) //It's outside, so make it go towards the build plate.
            {
                nodes_per_part[0][node.position] = p_node;
                continue;
            }
            /* Find which part this node is located in and group the nodes in
             * the same part together. Since nodes have a radius and the
             * avoidance areas are offset by that radius, the set of parts may
             * be different per node. Here we consider a node to be inside the
             * part that is closest. The node may be inside a bigger part that
             * is actually two parts merged together due to an offset. In that
             * case we may incorrectly keep two nodes separate, but at least
             * every node falls into some group.
             */
            coord_t closest_part_distance2 = std::numeric_limits<coord_t>::max();
            size_t closest_part = -1;
            for (size_t part_index = 0; part_index < parts.size(); part_index++)
            {
                constexpr bool border_result = true;
                if (parts[part_index].inside(node.position, border_result)) //If it's inside, the distance is 0 and this part is considered the best.
                {
                    closest_part = part_index;
                    closest_part_distance2 = 0;
                    break;
                }
                const ClosestPolygonPoint closest_point = PolygonUtils::findClosest(node.position, parts[part_index]);
                const coord_t distance2 = vSize2(node.position - closest_point.location);
                if (distance2 < closest_part_distance2)
                {
                    closest_part_distance2 = distance2;
                    closest_part = part_index;
                }
            }
            //Put it in the best one.
            nodes_per_part[closest_part + 1][node.position] = p_node; //Index + 1 because the 0th index is the outside part.
        }
        //Create a MST for every part.
        std::vector<MinimumSpanningTree> spanning_trees;
        for (const std::unordered_map<Point, Node*>& group : nodes_per_part)
        {
            std::unordered_set<Point> points_to_buildplate;
            for (const std::pair<Point, Node*>& entry : group)
            {
                points_to_buildplate.insert(entry.first); //Just the position of the node.
            }
            spanning_trees.emplace_back(points_to_buildplate);
        }

        for (size_t group_index = 0; group_index < nodes_per_part.size(); group_index++)
        {
            const MinimumSpanningTree& mst = spanning_trees[group_index];
            //In the first pass, merge all nodes that are close together.
            std::unordered_set<Node*> to_delete;
            for (const std::pair<Point, Node*>& entry : nodes_per_part[group_index])
            {
                Node* p_node = entry.second;
                Node& node = *p_node;
                if (to_delete.find(p_node) != to_delete.end())
                {
                    continue; //Delete this node (don't create a new node for it on the next layer).
                }
                const std::vector<Point>& neighbours = mst.adjacentNodes(node.position);
                if (neighbours.size() == 1 && vSize2(neighbours[0] - node.position) < maximum_move_distance * maximum_move_distance && mst.adjacentNodes(neighbours[0]).size() == 1) //We have just two nodes left, and they're very close!
                {
                    //Insert a completely new node and let both original nodes fade.
                    Point next_position = (node.position + neighbours[0]) / 2; //Average position of the two nodes.

                    const coord_t branch_radius_node = [&]() -> coord_t
                    {
                        if ((node.distance_to_top + 1) > tip_layers)
                        {
                             return branch_radius + branch_radius * (node.distance_to_top + 1) * diameter_angle_scale_factor;
                        }
                        else
                        {
                             return branch_radius * (node.distance_to_top + 1) / tip_layers;
                        }
                    }();
                    if (group_index == 0)
                    {
                        //Avoid collisions.
                        const coord_t maximum_move_between_samples = maximum_move_distance + radius_sample_resolution + 100; //100 micron extra for rounding errors.
                        PolygonUtils::moveOutside(volumes_.getAvoidance(branch_radius_node, layer_nr - 1), next_position, radius_sample_resolution + 100, maximum_move_between_samples * maximum_move_between_samples); //Some extra offset to prevent rounding errors with the sample resolution.
                    }
                    else
                    {
                        //Move towards centre of polygon.
                        const ClosestPolygonPoint closest_point_on_border = PolygonUtils::findClosest(node.position, volumes_.getInternalModel(branch_radius_node, layer_nr - 1));
                        const coord_t distance = vSize(node.position - closest_point_on_border.location);
                        //Try moving a bit further inside: Current distance + 1 step.
                        Point moved_inside = next_position;
                        PolygonUtils::ensureInsideOrOutside(volumes_.getInternalModel(branch_radius_node, layer_nr - 1), moved_inside, closest_point_on_border, distance + maximum_move_distance);
                        Point difference = moved_inside - node.position;
                        if(vSize2(difference) > maximum_move_distance * maximum_move_distance)
                        {
                            difference = normal(difference, maximum_move_distance);
                        }
                        next_position = node.position + difference;
                    }

                    const bool to_buildplate = !volumes_.getAvoidance(branch_radius_node, layer_nr - 1).inside(next_position);
                    Node* next_node = new Node(next_position, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, to_buildplate, p_node);
                    insertDroppedNode(contact_nodes[layer_nr - 1], next_node); //Insert the node, resolving conflicts of the two colliding nodes.

                    // Make sure the next pass doens't drop down either of these (since that already happened).
                    Node* neighbour = nodes_per_part[group_index][neighbours[0]];
                    node.merged_neighbours.push_front(neighbour);
                    to_delete.insert(neighbour);
                    to_delete.insert(p_node);
                }
                else if (neighbours.size() > 1) //Don't merge leaf nodes because we would then incur movement greater than the maximum move distance.
                {
                    //Remove all neighbours that are too close and merge them into this node.
                    for (const Point& neighbour : neighbours)
                    {
                        if (vSize2(neighbour - node.position) < maximum_move_distance * maximum_move_distance)
                        {
                            Node* neighbour_node = nodes_per_part[group_index][neighbour];
                            node.distance_to_top = std::max(node.distance_to_top, neighbour_node->distance_to_top);
                            node.support_roof_layers_below = std::max(node.support_roof_layers_below, neighbour_node->support_roof_layers_below);
                            node.merged_neighbours.push_front(neighbour_node);
                            node.merged_neighbours.insert_after(node.merged_neighbours.end(), neighbour_node->merged_neighbours.begin(), neighbour_node->merged_neighbours.end());
                            to_delete.insert(neighbour_node);
                        }
                    }
                }
            }
            //In the second pass, move all middle nodes.
            for (const std::pair<Point, Node*>& entry : nodes_per_part[group_index])
            {
                Node* p_node = entry.second;
                const Node& node = *p_node;
                if (to_delete.find(p_node) != to_delete.end())
                {
                    continue;
                }
                //If the branch falls completely inside a collision area (the entire branch would be removed by the X/Y offset), delete it.
                if (group_index > 0 && volumes_.getCollision(0, layer_nr).inside(node.position))
                {
                    const coord_t branch_radius_node = [&]() -> coord_t
                    {
                        if (node.distance_to_top > tip_layers)
                        {
                            return branch_radius + branch_radius * node.distance_to_top * diameter_angle_scale_factor;
                        }
                        else
                        {
                            return branch_radius * node.distance_to_top / tip_layers;
                        }
                    }();
                    const ClosestPolygonPoint to_outside = PolygonUtils::findClosest(node.position, volumes_.getCollision(0, layer_nr));
                    if (vSize2(node.position - to_outside.location) >= branch_radius_node * branch_radius_node) //Too far inside.
                    {
                        if (! support_rests_on_model)
                        {
                            unsupported_branch_leaves.push_front({ layer_nr, p_node });
                        }
                        continue;
                    }
                }
                Point next_layer_vertex = node.position;
                const std::vector<Point> neighbours = mst.adjacentNodes(node.position);
                if (neighbours.size() > 1 || (neighbours.size() == 1 && vSize2(neighbours[0] - node.position) >= maximum_move_distance * maximum_move_distance)) //Only nodes that aren't about to collapse.
                {
                    //Move towards the average position of all neighbours.
                    Point sum_direction(0, 0);
                    for (const Point& neighbour : neighbours)
                    {
                        sum_direction += neighbour - node.position;
                    }
                    if(vSize2(sum_direction) <= maximum_move_distance * maximum_move_distance)
                    {
                        next_layer_vertex += sum_direction;
                    }
                    else
                    {
                        next_layer_vertex += normal(sum_direction, maximum_move_distance);
                    }
                }

                const coord_t branch_radius_node = [&]() -> coord_t
                {
                    if ((node.distance_to_top + 1) > tip_layers)
                    {
                        return branch_radius + branch_radius * (node.distance_to_top + 1) * diameter_angle_scale_factor;
                    }
                    else
                    {
                        return branch_radius * (node.distance_to_top + 1) / tip_layers;
                    }
                }();
                if (group_index == 0)
                {
                    //Avoid collisions.
                    const coord_t maximum_move_between_samples = maximum_move_distance + radius_sample_resolution + 100; //100 micron extra for rounding errors.
                    PolygonUtils::moveOutside(volumes_.getAvoidance(branch_radius_node, layer_nr - 1), next_layer_vertex, radius_sample_resolution + 100, maximum_move_between_samples * maximum_move_between_samples); //Some extra offset to prevent rounding errors with the sample resolution.
                }
                else
                {
                    //Move towards centre of polygon.
                    const ClosestPolygonPoint closest_point_on_border = PolygonUtils::findClosest(next_layer_vertex, volumes_.getInternalModel(branch_radius_node, layer_nr - 1));
                    const coord_t distance = vSize(node.position - closest_point_on_border.location);
                    //Try moving a bit further inside: Current distance + 1 step.
                    Point moved_inside = next_layer_vertex;
                    PolygonUtils::ensureInsideOrOutside(volumes_.getInternalModel(branch_radius_node, layer_nr - 1), moved_inside, closest_point_on_border, distance + maximum_move_distance);
                    Point difference = moved_inside - node.position;
                    if(vSize2(difference) > maximum_move_distance * maximum_move_distance)
                    {
                        difference = normal(difference, maximum_move_distance);
                    }
                    next_layer_vertex = node.position + difference;
                }

                const bool to_buildplate = !volumes_.getAvoidance(branch_radius_node, layer_nr - 1).inside(next_layer_vertex);
                Node* next_node = new Node(next_layer_vertex, node.distance_to_top + 1, node.skin_direction, node.support_roof_layers_below - 1, to_buildplate, p_node);
                insertDroppedNode(contact_nodes[layer_nr - 1], next_node);
            }
        }

        // Prune all branches that couldn't find support on either the model or the buildplate (resulting in 'mid-air' branches).
        for (;! unsupported_branch_leaves.empty(); unsupported_branch_leaves.pop_back())
        {
            const auto& entry = unsupported_branch_leaves.back();
            Node* i_node = entry.second;
            for (size_t i_layer = entry.first; i_node != nullptr; ++i_layer, i_node = i_node->parent)
            {
                contact_nodes[i_layer].erase(i_node);
                to_free_node_set.insert(i_node);
                for (Node* neighbour : i_node->merged_neighbours)
                {
                    unsupported_branch_leaves.push_front({i_layer, neighbour});
                }
            }
        }

        const double progress_current = (contact_nodes.size() - layer_nr) * PROGRESS_WEIGHT_DROPDOWN;
        const double progress_total = contact_nodes.size() * PROGRESS_WEIGHT_DROPDOWN + contact_nodes.size() * PROGRESS_WEIGHT_AREAS;
        Progress::messageProgress(Progress::Stage::SUPPORT, progress_current, progress_total);
    }

    for (Node *node : to_free_node_set)
    {
        delete node;
    }
    to_free_node_set.clear();
}


Polygons getLineEndPointsPolyline(const Polygons& poly,coord_t inset,coord_t maxSegLength){
	std::deque<std::deque<Point>> result;

	std::function<void(Point)> insertPoint=[&](Point p) {
		if(result.empty()){
			result.emplace_back(std::deque<Point>{p});
			return;
		}
		coord_t pos=-1;
		bool front=false;
		coord_t closest=std::numeric_limits<coord_t>::max();

		for(size_t i=0;i<result.size();i++){
			std::deque<Point> line=result[i];
			if(vSize2(line.front()-p)<closest){
				closest=vSize2(line.front()-p);
				pos=i;
				front=true;
			}
			if(vSize2(line.back()-p)<closest){
				closest=vSize2(line.back()-p);
				pos=i;
				front=false;
			}
		}

		if(closest<maxSegLength*maxSegLength){
			if(front)
				result[pos].emplace_front(p);
			else
				result[pos].emplace_back(p);
		}
		else
			result.emplace_back(std::deque<Point>{p});

	};

	if(!poly.empty())
		for(auto part : poly){


			Point p1 =part[0];
			Point p2 =part[part.size()-1];

			Point v12 = p2-p1;
			Point v21= p1-p2;

			if(vSize(v12)<=inset*2)// if we would inset from both sides nothing is left or the distance is so small that it would be useless
				continue;
			if(vSize(v12)<=inset*4){
				insertPoint((p1+p2)/2);
				continue;
			}

			double scale = (1.0*inset)/vSize(v12);
			p1=p1+v12*scale; // move points inside by inset
			p2=p2+v21*scale;
			insertPoint(p1);
			insertPoint(p2);

		}

	Polygons resultPoly;

	for(std::deque<Point> line:result){

		Polygon resLine;
		for(Point p:line)
			resLine.add(p);
		resultPoly.add(resLine);
	}
	return resultPoly;

}


std::vector<TreeSupport::LineInformation> TreeSupport::convertLinesToInternal(Polygons polylines,coord_t layer_nr){
	std::vector<LineInformation> result;

	std::vector<std::pair<AABB,Polygons>> avoid_to_bp;
	std::vector<std::pair<AABB,Polygons>> avoid_to_model;
	std::vector<std::pair<AABB,Polygons>> collision;
	if(config.support_rests_on_model){
		for(auto part: volumes_.getCollision(config.getRadius(1), layer_nr).splitIntoParts(false)){
			collision.emplace_back(AABB(part),part);
		}
		for(auto part: volumes_.getAvoidance(config.getRadius(1), layer_nr,false,true).splitIntoParts(false)){
			avoid_to_model.emplace_back(AABB(part),part);
		}
	}
	for(auto part: volumes_.getAvoidance(config.getRadius(1), layer_nr,false,false).splitIntoParts(false)){
		avoid_to_bp.emplace_back(AABB(part),part);
	}

	std::function<bool(Point p,std::vector<std::pair<AABB,Polygons>>)> fastInside=[&](Point p,std::vector<std::pair<AABB,Polygons>> area) {

		for(std::pair<AABB,Polygons> part:area){
			if(part.first.contains(p)&&part.second.inside(p, true))
				return true;
		}

		return false;
	};


	// we check if the position is valid, if it is NOT we delete that point
	for(auto line:polylines){
		LineInformation resLine;
		for(Point p:line){
			if(!fastInside(p,avoid_to_bp)){
				resLine.emplace_back(p,LineStatus::TO_BP);
			}
			else if(config.support_rests_on_model&&!fastInside(p,avoid_to_model)){
				resLine.emplace_back(p,LineStatus::TO_MODEL_GRACIOUS);
			}
			else if(config.support_rests_on_model&&!fastInside(p,collision)){
				resLine.emplace_back(p,LineStatus::TO_MODEL);
			}
			else{
				if(!resLine.empty()){
					result.emplace_back(resLine);
					resLine.clear();
				}
			}

		}
		if(!resLine.empty()){
			result.emplace_back(resLine);
			resLine.clear();
		}
	}

	return result;

}

Polygons TreeSupport::convertInternalToLines(std::vector<TreeSupport::LineInformation> lines){
	Polygons result;

	for(LineInformation line:lines){
		Polygon path;
		for(auto pointData:line){
			path.add(pointData.first);
		}
		result.add(path);
	}
	return result;
}


//splits the lines into lines that can be migrated down and lines that have to go into the tree algorithm
std::pair<std::vector<TreeSupport::LineInformation>,std::vector<TreeSupport::LineInformation>> TreeSupport::splitLines(std::vector<TreeSupport::LineInformation> lines,size_t current_layer,size_t ddt){//assumes all Points on the current line are valid

	std::vector<LineInformation> keep(1);
	std::vector<LineInformation> setFree(1);
	enum STATE{keeping,freeing};

	std::vector<std::pair<AABB,Polygons>> avoid_to_bp;
	std::vector<std::pair<AABB,Polygons>> avoid_to_model;
	std::vector<std::pair<AABB,Polygons>> collision;
	if(config.support_rests_on_model){
		for(auto part: volumes_.getCollision(config.getRadius(ddt+1), current_layer-1).splitIntoParts(false)){
			collision.emplace_back(AABB(part),part);
		}
		for(auto part: volumes_.getAvoidance(config.getRadius(ddt+1), current_layer-1,false,true).splitIntoParts(false)){
			avoid_to_model.emplace_back(AABB(part),part);
		}
	}
	for(auto part: volumes_.getAvoidance(config.getRadius(ddt+1), current_layer-1,false,false).splitIntoParts(false)){
		avoid_to_bp.emplace_back(AABB(part),part);
	}

	std::function<bool(Point p,std::vector<std::pair<AABB,Polygons>>)> fastInside=[&](Point p,std::vector<std::pair<AABB,Polygons>> area) {

		for(std::pair<AABB,Polygons> part:area){
			if(part.first.contains(p)&&part.second.inside(p, true))
				return true;
		}

		return false;
	};

	std::function<bool(std::pair<Point,LineStatus>)> evaluatePoint=[&](std::pair<Point,LineStatus> p) {
		if(!fastInside(p.first,avoid_to_bp)){
			return true;
		}
		if(config.support_rests_on_model&&p.second!=LineStatus::TO_BP){

			if( p.second==LineStatus::TO_MODEL_GRACIOUS){
				return !fastInside(p.first,avoid_to_model);

			}
			else{
				return !fastInside(p.first,collision);
			}

		}

		return false;
	};

	for(std::vector<std::pair<Point,LineStatus>> line:lines){
		STATE current =keeping;
		LineInformation resLine;
		for(std::pair<Point,LineStatus> me:line){

			if(evaluatePoint(me)){
				if(keeping!=current){
					if(!resLine.empty()){
						setFree.emplace_back(resLine);
						resLine.clear();
					}
					current=keeping;
				}
				resLine.emplace_back(me);
			}
			else{
				if(freeing!=current){
					if(!resLine.empty()){
						keep.emplace_back(resLine);
						resLine.clear();
					}
					current=freeing;
				}
				resLine.emplace_back(me);
			}

		}
		if(!resLine.empty()){
			if(current==keeping){
				keep.emplace_back(resLine);
			}
			else{
				setFree.emplace_back(resLine);
			}
		}
	}

	return std::pair<std::vector<std::vector<std::pair<Point,TreeSupport::LineStatus>>>,std::vector<std::vector<std::pair<Point,TreeSupport::LineStatus>>>> (keep,setFree);


}


void writePolylines(SVG& svg,Polygons polylines,SVG::Color color){
	for(auto path :polylines){
		if(path.size()==0)
			continue;
		Point before = path[0];
		for(size_t i=1;i<path.size();i++){
			svg.writeLine(before, path[i], color, 2);
			before=path[i];
		}
	}
}

// returns the delta to 180°
double getAngle180(const Point& a, const Point& b, const Point& c){
	double leftAngle = LinearAlg2D::getAngleLeft(a, b, c);

	if(leftAngle>M_PI)
		return std::abs(leftAngle-M_PI);
	else
		return std::abs((M_PI*2-leftAngle)-M_PI);

}

Polygons invertLineDirections(Polygons polylines){
	Polygons result;
	for(auto line:polylines){
		Polygon resline;
		for(int i=line.size()-1;i>=0;i--){
			resline.add(line[i]);
		}
		result.add(resline);
	}
	return result;
}


Polygons projectInwardsFromOutline(Polygons outline,const coord_t distance,const coord_t length,coord_t layer_nr){ //Polygons splited bei plane meanign the 0 polygon has 0 inside them while the 1 polygon can have 1 planes further in

	Polygons to_small;

	//AABB maxBounds(Point(0,0),Point(300000,300000) );
	//SVG svg(std::to_string(layer_nr)+"  "+ std::to_string(length)+"space.svg", maxBounds,0.1,SVG::Color::NONE);
	//svg.writeAreas(outline,SVG::Color::NONE, SVG::Color::BLACK, 2);
	//printf("distance %d length %d\n",distance,length);
	const coord_t minAngleDistance=distance/2;
	constexpr double force_place_angle=M_PI_4;
	constexpr double accumulate_place_angle=M_PI_2; // todo change to LIFO buffer of old angle by distance
	const Polygons outline_inset=outline.offset(-length);
	//svg.writeAreas(outline_inset,SVG::Color::NONE, SVG::Color::BLUE, 2);
	Polygons resultingLines;
	//printf("%s\n",getPolygonAsString(outline).c_str());
	for(ClipperLib::Path part : outline){
		Polygons part_lines;

		std::function<Point(Point,Point,bool)> getNormal=[&](Point a,Point b,bool inDirection){
			Point segment=a-b;
			Point normal=inDirection?Point(-segment.Y,segment.X):Point(segment.Y,-segment.X);

			return normal;
		};
		coord_t initalPoint_used_distance =-1;
		coord_t used_distance=0;
		double accumulatedAngle=0;
		Point initNormal=getNormal(part[0],part[1],false);
		initNormal=initNormal*((10.0)/vSize(initNormal));
		bool inDirectionInside=outline.inside((part[0]+part[1])/2+initNormal, false);
		coord_t tba_start=0;
		coord_t insertedPoints=0;
		coord_t minX=part[0].X,minY=part[0].Y,maxX=part[0].X,maxY=part[0].Y;

		//before => the vertex id of the start of the line before my current line and after the start of the current line; after-before has to be 0 if we place a normal on a line or 1 if we place it on an angled endpoint
		std::function<void(Point,coord_t,coord_t)> InsertPoint=[&](Point ins,coord_t before,coord_t after){
			if(initalPoint_used_distance==-1)
				initalPoint_used_distance=used_distance;
			Point me = part[(after)%part.size()];
			Point segBeg=part[(part.size()+before)%part.size()];
			Point segEnd =part[(after+1)%part.size()];

			double miter_overshoot=0;
			if(before!=after){
				miter_overshoot=std::min(std::sqrt(2/(1-(std::cos(M_PI+getAngle180(segBeg, me,segEnd)))))*length,2.0*length)-length;
				if(me==segBeg)
					segBeg=part[(part.size()+before-1)%part.size()];
				if(vSize2(me-segBeg)<vSize2(me-segEnd)){ // to get an optimal angle we ensure both points of the virtual segment have equal distance from our point
					segBeg=me+(segBeg-me)*(vSize(me-segEnd)*1.0/vSize(me-segBeg));
				}
				else{
					segEnd=me+(segEnd-me)*(vSize(me-segBeg)*1.0/vSize(me-segEnd));
				}

			}
			Point normal=getNormal(segEnd,segBeg,inDirectionInside);
			Point normalResult = ins+normal*((1.0*length+miter_overshoot+10)/vSize(normal));
			Point result=normalResult;
			PolygonUtils::moveInside(outline_inset,result,0);
			if(vSize(result-ins)>(length+miter_overshoot)*1.3){//todo math
				result = ins;
				PolygonUtils::moveInside(outline_inset,result,0);
				if(vSize(result-ins)>(length+miter_overshoot)*1.3){
					result= normalResult;
				}
			}

			part_lines.addLine(ins, result);
			//svg.writeLine(ins, result,debug?SVG::Color::GREEN: SVG::Color::RED, 2);
			//svg.nextLayer();
			insertedPoints++;
			accumulatedAngle=0;

		};

		std::function<void(coord_t,coord_t)> PlaceNewLines=[&](coord_t to,bool ommitLast) {


			if(initalPoint_used_distance==-1)
				initalPoint_used_distance=distance;

			coord_t relevant_used=0;
			Point insert=Point(-1,-1);
			coord_t at=0;

			for(coord_t pos=tba_start;pos<to;pos++){
				Point me =part[(part.size()+pos)%part.size()];
				Point next=part[(part.size()+pos+1)%part.size()];
				accumulatedAngle+=getAngle180(part[(pos+part.size()-1)%part.size()], me, next);
				while(vSize(me-next)+relevant_used>=distance){
						double scale = (1.0*(distance-relevant_used))/(vSize(me-next));
						me= me + (next-me)*scale;
						relevant_used=0;
						if(Point(-1,-1)!=insert)
							InsertPoint(insert,at,at);

						insert=me;
						at=pos;
				}

			relevant_used+=vSize(me-next);
			}
			if(!ommitLast&&insert!=Point(-1,-1))
				InsertPoint(insert,at,at);
		};

		coord_t over_max_used=0;
		size_t line_position;
		for(line_position=0;line_position<=part.size()||over_max_used>used_distance;line_position++){


			if(line_position==part.size()){

				over_max_used=used_distance+initalPoint_used_distance-distance;
				if(over_max_used<used_distance) // we are closer than minDist
					break;
			}

			Point me=part[line_position%part.size()];
			Point next=part[(line_position+1)%part.size()];

			// here we do our bounding box inplace to be faster
			if(me.X<minX)
				minX=me.X;
			if(me.Y<minY)
				minY=me.Y;
			if(me.X>maxX)
				maxX=me.X;
			if(me.Y>maxY)
				maxY=me.Y;

			if(line_position>=part.size()&&used_distance+vSize(me-next)>over_max_used){ // we need to limit next

				next=me+(next-me)*(1.0*over_max_used - used_distance)/vSize(me-next);//doing the last step
				used_distance=over_max_used; // prevents offbyone
			}

			double stepAngle= getAngle180(part[(line_position+part.size()-1)%part.size()], me, next); // todo update rest with +size
			accumulatedAngle+=stepAngle;
			bool forceAnglePlace=(stepAngle>force_place_angle||accumulatedAngle>accumulate_place_angle)&&(used_distance>minAngleDistance)&&(!over_max_used||over_max_used-used_distance<minAngleDistance); // if we are zo close that we are not allowed to place something to support our angle we dont even try
			if(forceAnglePlace){
				PlaceNewLines(line_position,true);
				InsertPoint(me,coord_t(line_position)-1,line_position);
				tba_start=line_position;
				used_distance=0;
				if(over_max_used)
					break;
			}
			used_distance+=vSize(me-next);

		}

		PlaceNewLines(line_position-1,false);

		if(insertedPoints<3){ // our part is very small so we just add an x through it ensuring at least a few support points
			if(part.size()<3)
				continue;
			part_lines.clear();
			if(inDirectionInside){ //if true than we have a hole
				to_small.add(part);
			}
			else{

				coord_t sideEnlarged=sqrt((length*length)/2);
				resultingLines.addLine(Point(minX-sideEnlarged,maxY+sideEnlarged),Point(maxX+sideEnlarged,minY-sideEnlarged));
				resultingLines.addLine(Point(minX-sideEnlarged,minY-sideEnlarged),Point(maxX+sideEnlarged,maxY+sideEnlarged));
				// added twice as enlarge only enlarges the begin of a line
				resultingLines.addLine(Point(maxX+sideEnlarged,minY-sideEnlarged),Point(minX-sideEnlarged,maxY+sideEnlarged));
				resultingLines.addLine(Point(maxX+sideEnlarged,maxY+sideEnlarged),Point(minX-sideEnlarged,minY-sideEnlarged));
			}

		}
		resultingLines.add(part_lines);

	}
	if(!to_small.empty()){

		Polygons next_iter=to_small.offset(length).unionPolygons();
		//svg.nextLayer();
		//svg.writeAreas(next_iter,SVG::Color::NONE, SVG::Color::RED, 2);
		Polygons extra_lines=projectInwardsFromOutline(next_iter, distance+1, length+1,layer_nr);
		resultingLines.add(invertLineDirections(extra_lines));
	}
	//svg.nextLayer();
	//writePolylines(svg, resultingLines, SVG::Color::ORANGE);
	return resultingLines;
}

//ensures that every line segment is at most distance in legnth
Polygons ensureMaximumDistancePolyline (Polygons input, coord_t distance){
	Polygons result;
	for(auto part:input){
		if(part.size()==0)
			continue;
		Polygon line;
		line.add(part[0]);
		coord_t used_distance=0;
		for(size_t pos=0;pos+1<part.size();pos++){
			Point me =part[(part.size()+pos)%part.size()];
			Point next=part[(part.size()+pos+1)%part.size()];
			coord_t next_step_length;
			do{
				next_step_length=vSize(me-next);
				if(next_step_length+used_distance>=distance){
					double scale = (1.0*(distance-used_distance))/(next_step_length);
					me= me + (next-me)*scale;
					used_distance=0;
					line.add(me);
					continue;
				}
				used_distance+=next_step_length;
			}
			while(next_step_length>distance);
		}
		line.add(part.back());
		result.add(line);

	}
	return result;
}


//adds the implizit line from the last vertex to the first explicitly
Polygons toPolylines(Polygons poly){
	Polygons result;
	for(auto path :poly){
		Polygon part;
		for(size_t i=0;i<path.size();i++){
			part.add(path[i]);
		}
		part.add(path[0]);
		result.add(part);
	}
	return result;
}
//Enlarges a polyline; assumes line only consists of 2 points
Polygons enlarge2PointPolylines(Polygons& polylines,coord_t outset){

	Polygons result;

	for(auto path:polylines){
		Point normal=path[1]-path[0];
		Point resIns=path[0]-normal*((1.0*outset)/vSize(normal));
		result.addLine(resIns, path[1]);
	}
	return result;
}

void TreeSupport::generateInitalAreas(const SliceMeshStorage &mesh,std::vector<std::map<SupportElement*,Polygons*>>& moveBounds,SliceDataStorage &storage){

	Polygon base_circle;

	const int baseRadius=10;
	for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++) {
		const AngleRadians angle = static_cast<double>(i) / CIRCLE_RESOLUTION
				* TAU;
		base_circle.emplace_back(cos(angle) * baseRadius, sin(angle) * baseRadius);
	}

	const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
	const size_t z_distance_top_layers =( round_up_divide(z_distance_top, config.layer_height) + 1); //Support must always be 1 layer below overhang.
	const coord_t support_line_width=mesh.settings.get<coord_t>("support_line_width");
	const coord_t step_length = mesh.settings.get<coord_t>("support_tree_contact_line_distance");
	const coord_t walls =mesh.settings.get<size_t>("wall_line_count");
	const coord_t infill_walls=mesh.settings.get<size_t>("skin_outline_count");
	const bool use_smart_wall=mesh.settings.get<bool>("support_tree_use_smart_wall");
	const bool only_gracious=mesh.settings.get<bool>("support_tree_only_gracious_to_model");
	coord_t line_width; // todo maybe properly in different variables: skin and wall
	std::vector<AngleDegrees> skin_angles;
	coord_t skin_line_width;
	coord_t wall_line_width;
	skin_angles= mesh.settings.get<std::vector<AngleDegrees>>("skin_angles");

	skin_line_width=mesh.settings.get<coord_t>("skin_line_width");
	wall_line_width=(mesh.settings.get<coord_t>("wall_line_width_0")+mesh.settings.get<coord_t>("wall_line_width_x")*(walls-1))/walls;
	line_width=(mesh.settings.get<coord_t>("wall_line_width_0")+mesh.settings.get<coord_t>("wall_line_width_x")*(walls-1)+skin_line_width*infill_walls)/(walls+infill_walls);

	const coord_t connect_length =2*(config.getRadius(1,0)+config.maximum_move_distance_slow);
	const coord_t outset=mesh.settings.get<coord_t>("support_tree_support_line_outset");
	const bool easyRemove=mesh.settings.get<bool>("support_tree_easy_remove")&&outset>=support_line_width;

	if(skin_angles.size() == 0){
		skin_angles.push_back(45);
		skin_angles.push_back(135);
	}

	logDebug("Distance between points of lines of inital influene areas %lld \n",connect_length);
	std::vector<std::unordered_set<Point>> alreadyInserted(mesh.overhang_areas.size() - z_distance_top_layers);

	if(mesh.overhang_areas.size()<=z_distance_top_layers)
		return;

	//for (size_t layer_nr = 1; layer_nr < mesh.overhang_areas.size() - z_distance_top_layers;layer_nr++) {
#pragma omp parallel for
	for (coord_t layer_nr =  mesh.overhang_areas.size() - z_distance_top_layers -1 ; layer_nr >=1;layer_nr--) {
		if(mesh.overhang_areas[layer_nr+ z_distance_top_layers].empty())
			continue;

		const Polygons outline =storage.getLayerOutlines(layer_nr+ z_distance_top_layers, false, false, false);

		Polygons relevantForbidden=(config.support_rests_on_model?(only_gracious?volumes_.getAvoidance(config.getRadius(1),layer_nr,false,true):volumes_.getCollision(config.getRadius(1), layer_nr)):volumes_.getAvoidance(config.getRadius(1),layer_nr));

		Polygons overhang = mesh.overhang_areas[layer_nr+ z_distance_top_layers].difference(relevantForbidden);

		if(overhang.empty())
			continue;

		Polygons skin=outline.offset(-(line_width*(walls+infill_walls-0.5))).intersection(overhang);// we can only check to overhang as we dont support concentric, if one want concentric he shall pick triangle or grid infill or similar
		Polygons overhangOutset=overhang.offset(outset).difference(relevantForbidden);

		Polygons gaps;

		//as we effectivly use lines to place our supportPoints we may use the Infill class for it, while not made for it it works perfect
		const EFillMethod pattern=mesh.settings.get<EFillMethod>("support_tree_contact_node_pattern"); //!< the space filling pattern of the infill to generate
		constexpr bool zig_zaggify=false; //!< Whether to connect the end pieces of the support lines via the wall
		constexpr bool connect_polygons=false; //!< Whether to connect as much polygons together into a single path
		const Polygons& in_outline=use_smart_wall?skin.offset(-line_width):overhangOutset; //!< a reference polygon for getting the actual area within which to generate infill (see outline_offset)
		//constexpr coord_t outline_offset=0; //!< Offset from Infill::in_outline to get the actual area within which to generate infill
		constexpr coord_t infill_line_width=10; //!< The line width of the infill lines to generate
		const coord_t line_distance=step_length; //!< The distance between two infill lines / polygons
		constexpr coord_t infill_overlap=0; //!< the distance by which to overlap with the actual area within which to generate infill
		constexpr size_t infill_multiplier=1; //!< the number of infill lines next to each other
		const AngleDegrees fill_angle=skin_angles[(layer_nr+z_distance_top_layers)%skin_angles.size()]+90; //!< for linear infill types: the angle of the infill lines (or the angle of the grid)
		const coord_t z=layer_nr+z_distance_top_layers; //!< height of the layer for which we generate infill
		const coord_t shift=step_length/2; //!< shift of the scanlines in the direction perpendicular to the fill_angle
		constexpr size_t wall_line_count=0; //!< Number of walls to generate at the boundary of the infill region, spaced \ref infill_line_width apart ;;  We dont use the walls here as they use miter offset causing edges ???
		const Point infill_origin=overhang[0][0]; //!< origin of the infill pattern
		Polygons* perimeter_gaps=&gaps; //!< (optional output) The areas in between consecutive insets when Concentric infill is used.
		constexpr bool connected_zigzags=false; //!< (ZigZag) Whether endpieces of zigzag infill should be connected to the nearest infill line on both sides of the zigzag connector
		constexpr bool use_endpieces=false; //!< (ZigZag) Whether to include endpieces: zigzag connector segments from one infill line to itself
		constexpr bool skip_some_zags=false;  //!< (ZigZag) Whether to skip some zags
		constexpr size_t zag_skip_count=3;  //!< (ZigZag) To skip one zag in every N if skip some zags is enabled
		constexpr coord_t pocket_size=0; //!< The size of the pockets at the intersections of the fractal in the cross 3d pattern

		Infill inf_skin(pattern, zig_zaggify, connect_polygons, in_outline, -line_width/2 , infill_line_width,line_distance , infill_overlap, infill_multiplier, fill_angle, z, shift, wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size);
		Polygons areas_skin;
		Polygons lines_skin;
		inf_skin.generate(areas_skin, lines_skin, storage.support.cross_fill_provider,&mesh);
		/*
		AABB maxBounds(Point(0,0),Point(300000,300000) );
		SVG svg(std::to_string(layer_nr)+"de_gen.svg", maxBounds,0.1,SVG::Color::NONE);
		svg.writeAreas(skin, SVG::Color::NONE, SVG::Color::BLUE, 2);
		svg.nextLayer();
		svg.writeAreas(outline, SVG::Color::NONE, SVG::Color::GREEN, 2);
		svg.nextLayer();
		svg.writeAreas(overhang, SVG::Color::NONE, SVG::Color::RED, 2);
		svg.nextLayer();
		svg.writeAreas(relevantForbidden, SVG::Color::NONE, SVG::Color::MAGENTA, 2);
		svg.nextLayer();
		writePolylines(svg, lines_skin, SVG::Color::ORANGE);
		svg.nextLayer();
		svg.writeAreas(outline.offset(outset-line_width/2), SVG::Color::NONE, SVG::Color::YELLOW, 2);
		svg.nextLayer();
		svg.writeAreas(overhangOutset, SVG::Color::NONE, SVG::Color::RED, 2);
		svg.nextLayer();
		svg.writeAreas(volumes_.getCollision(0, layer_nr), SVG::Color::NONE, SVG::Color::MAGENTA, 2);
		svg.nextLayer();


		svg.writeAreas(skin, SVG::Color::NONE, SVG::Color::BLACK, 2);
		svg.nextLayer();
*/
		gaps.clear();
		Polygons normalLines;
		if(use_smart_wall){
			normalLines=projectInwardsFromOutline(outline, step_length, (walls+infill_walls)*line_width+1.25*line_width,layer_nr);//outset
			normalLines=overhangOutset.intersectionPolyLines(normalLines);
			normalLines=enlarge2PointPolylines(normalLines, outset);

			for(PolygonsPart part:skin.offset(-line_width).splitIntoParts(false)){ // we simulate the top lines if line skin is used to correctly pace support where the lines begin
				Polygons areas_skinnwall;
				Polygons lines_skinnwall;
				Infill inf_skinnwall(EFillMethod::LINES, zig_zaggify, connect_polygons, part, -line_width/2, line_width, line_width, infill_overlap, infill_multiplier, (fill_angle+90), z, shift, wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size);
				inf_skinnwall.generate(areas_skinnwall, lines_skinnwall, storage.support.cross_fill_provider,&mesh);

				normalLines.add(getLineEndPointsPolyline(lines_skinnwall, 0.5*line_width,line_width*4)); //todo make values configurable; How ???
			}
		}

		normalLines.add(lines_skin);
		normalLines.add(toPolylines(areas_skin));

		if(easyRemove&&outset-line_width/2>0&&use_smart_wall){
			// only do if skin
			Polygons outerRemove=overhang.offset(-round_up_divide(walls, 2)*line_width);
			Polygons ezyRem=toPolylines(outerRemove.offset(round_up_divide(walls, 2)*line_width+(outset-line_width/2)));
			ezyRem=outline.differencePolyLines(ezyRem);
			normalLines.add(ezyRem);
		}

		normalLines=overhangOutset.intersectionPolyLines(normalLines);

		std::function<void(std::pair<Point,LineStatus>,size_t,coord_t)> addToData=[&](std::pair<Point,LineStatus> p,size_t ddt, coord_t insert_layer) {

			bool to_bp=p.second==LineStatus::TO_BP;
			bool gracious = to_bp|| p.second==LineStatus::TO_MODEL_GRACIOUS;
			if(!config.support_rests_on_model&&!to_bp){
				return;
			}
			Polygon circle;
			Polygon outerCircle;
			for (Point corner : base_circle){
				circle.add(p.first + corner);
				outerCircle.add(p.first +corner*(10+((volumes_.ceilRadius(config.getRadius(ddt))*1.0)/baseRadius))); // adds 100 microns more to the radius as otherwise the draw areas may fail to ovalize
			}
			//svg.writePoint(p, false, ddt, SVG::Color::GREEN);
#pragma omp critical(moveBounds)
			{
				if(!alreadyInserted[insert_layer].count(p.first/((line_width+1)/10))){//we normalize the point a bit to also catch points which are so close that inserting it would achieve nothing
					alreadyInserted[insert_layer].emplace(p.first/((line_width+1)/10));
					SupportElement* elem= new SupportElement(ddt,insert_layer,p.first,to_bp,gracious); // +1 as we otherwise would
					elem->area=new Polygons(circle.offset(0));
					Polygons* outerWallArea=new Polygons(outerCircle.offset(0)); // technically wrong by up to base_radius, but i dont care about 10 microns
					moveBounds[insert_layer].emplace(elem,outerWallArea);
				}
			}
			//svg.writeAreas(*elem->area, SVG::Color::NONE, SVG::Color::BLUE, 2);
		};
		//svg.nextLayer();

		Polygons linesPoly=ensureMaximumDistancePolyline(normalLines, connect_length);

		linesPoly=relevantForbidden.offset(5).differencePolyLines(linesPoly); // offset 5 to avoid rounding errors

		std::vector<LineInformation> linesInt=convertLinesToInternal(linesPoly, layer_nr);


		for(size_t i=0;i<config.dont_move_until_ddt&&layer_nr-i>=1;i++){

			Polygons currentLines=convertInternalToLines(linesInt);
			std::pair<std::vector<LineInformation>,std::vector<LineInformation>> splits=splitLines(linesInt, layer_nr-i, i+1);

			linesInt=splits.first;
			for(auto line :splits.second)
				for(auto p:line){
					addToData(p,i+1,layer_nr-i);
				}

			Polygons supArea =currentLines.offsetPolyLineRound(config.getRadius(i+1, 0), ClipperLib::jtRound);
#pragma omp critical(precalculatedSupportLayers)
{
			if(i<config.support_roof_layers){
				storage.support.supportLayers[layer_nr-i].support_roof.add(supArea);
			}
			else{
				precalculatedSupportLayers[layer_nr-i].add(supArea);
			}
}

		}

		if(static_cast<coord_t>(config.dont_move_until_ddt)<layer_nr)
			for(auto line:linesInt){
				for(auto p:line)
					addToData(p,config.dont_move_until_ddt+(config.dont_move_until_ddt!=0?0:1),layer_nr-config.dont_move_until_ddt);
			}
		else{
			Polygons currentLines=convertInternalToLines(linesInt);
			precalculatedSupportLayers[0].add(currentLines.offsetPolyLineRound(config.getRadius(layer_nr+1, 0), ClipperLib::jtRound));
			Polygons supArea =currentLines.offsetPolyLineRound(config.getRadius(layer_nr+1, 0), ClipperLib::jtRound);
#pragma omp critical(precalculatedSupportLayers)
{
			if(layer_nr<static_cast<coord_t>(config.support_roof_layers)){
				storage.support.supportLayers[0].support_roof.add(supArea);
			}
			else{
				precalculatedSupportLayers[0].add(supArea);
			}
}
		}

	}

}

void TreeSupport::generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::unordered_set<TreeSupport::Node*>>& contact_nodes)
{
    const coord_t point_spread = mesh.settings.get<coord_t>("support_tree_branch_distance");

    //First generate grid points to cover the entire area of the print.
    AABB bounding_box = mesh.bounding_box.flatten();
    // We want to create the grid pattern at an angle, so compute the bounding
    // box required to cover that angle.
    // Rotation of 22 degrees provides better support of diagonal lines.
    constexpr double rotate_angle = 22.0 / 180.0 * M_PI;
    const Point bounding_box_size = bounding_box.max - bounding_box.min;

    // Store centre of AABB so we can relocate the generated points
    const auto centre = bounding_box.getMiddle();
    const auto sin_angle = std::sin(rotate_angle);
    const auto cos_angle = std::cos(rotate_angle);
    // Calculate the dimensions of the AABB of the mesh AABB after being rotated
    // by `rotate_angle`. Halve the dimensions since we'll be using it as a +-
    // offset from the centre of `bounding_box`.
    // This formulation will only work with rotation angles <90 degrees. If the
    // rotation angle becomes a user-configurable value then this will need to
    // be changed
    const auto rotated_dims = Point(
        bounding_box_size.X * cos_angle + bounding_box_size.Y * sin_angle,
        bounding_box_size.X * sin_angle + bounding_box_size.Y * cos_angle) / 2;

    std::vector<Point> grid_points;
    for (auto x = -rotated_dims.X; x <= rotated_dims.X; x += point_spread)
    {
        for (auto y = -rotated_dims.Y; y <= rotated_dims.Y; y += point_spread)
        {
            // Construct a point as an offset from the mesh AABB centre, rotated
            // about the mesh AABB centre
            const auto pt = rotate(Point(x, y), rotate_angle) + centre;
            // Only add to grid points if we have a chance to collide with the
            // mesh
            if (bounding_box.contains(pt))
            {
                grid_points.push_back(pt);
            }
        }
    }

    const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
    const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
    const size_t z_distance_top_layers = round_up_divide(z_distance_top, layer_height) + 1; //Support must always be 1 layer below overhang.
    const size_t support_roof_layers = [&]() -> size_t
    {
        if (mesh.settings.get<bool>("support_roof_enable"))
        {
            return round_divide(mesh.settings.get<coord_t>("support_roof_height"), mesh.settings.get<coord_t>("layer_height")); //How many roof layers, if roof is enabled.
        }
        else
        {
            return 0;
        }
    }();
    const coord_t half_overhang_distance = tan(mesh.settings.get<AngleRadians>("support_angle")) * layer_height / 2;
    for (size_t layer_nr = 1; static_cast<int>(layer_nr) < static_cast<int>(mesh.overhang_areas.size()) - static_cast<int>(z_distance_top_layers); layer_nr++)
    {
        const Polygons& overhang = mesh.overhang_areas[layer_nr + z_distance_top_layers];
        if (overhang.empty())
        {
            continue;
        }

        for (const ConstPolygonRef overhang_part : overhang)
        {
            AABB overhang_bounds(overhang_part); //Pre-generate the AABB for a quick pre-filter.
            overhang_bounds.expand(half_overhang_distance); //Allow for points to be within half an overhang step of the overhang area.
            bool added = false; //Did we add a point this way?
            for (Point candidate : grid_points)
            {
                if (overhang_bounds.contains(candidate))
                {
                    constexpr coord_t distance_inside = 0; //Move point towards the border of the polygon if it is closer than half the overhang distance: Catch points that fall between overhang areas on constant surfaces.
                    PolygonUtils::moveInside(overhang_part, candidate, distance_inside, half_overhang_distance * half_overhang_distance);
                    constexpr bool border_is_inside = true;
                    if (overhang_part.inside(candidate, border_is_inside) && !volumes_.getCollision(0, layer_nr).inside(candidate, border_is_inside))
                    {
                        constexpr size_t distance_to_top = 0;
                        constexpr bool to_buildplate = true;
                        Node* contact_node = new Node(candidate, distance_to_top, (layer_nr + z_distance_top_layers) % 2, support_roof_layers, to_buildplate, Node::NO_PARENT);
                        contact_nodes[layer_nr].insert(contact_node);
                        added = true;
                    }
                }
            }
            if (!added) //If we didn't add any points due to bad luck, we want to add one anyway such that loose parts are also supported.
            {
                Point candidate = bounding_box.getMiddle();
                PolygonUtils::moveInside(overhang_part, candidate);
                constexpr size_t distance_to_top = 0;
                constexpr bool to_buildplate = true;
                Node* contact_node = new Node(candidate, distance_to_top, layer_nr % 2, support_roof_layers, to_buildplate, Node::NO_PARENT);
                contact_nodes[layer_nr].insert(contact_node);
            }
        }
        for (const ConstPolygonRef overhang_part : overhang)
        {
            if (overhang_part.area() < 0)
            {
                for (auto iter = contact_nodes[layer_nr].begin(); iter != contact_nodes[layer_nr].end(); )
                {
                    if (overhang_part.inside((*iter)->position))
                    {
                        iter = contact_nodes[layer_nr].erase(iter);
                    }
                    else
                    {
                        ++iter;
                    }
                }
            }
        }
    }
}

void TreeSupport::insertDroppedNode(std::unordered_set<Node*>& nodes_layer, Node* p_node)
{
    std::unordered_set<Node*>::iterator conflicting_node_it = nodes_layer.find(p_node);
    if (conflicting_node_it == nodes_layer.end()) //No conflict.
    {
        nodes_layer.insert(p_node);
        return;
    }

    Node* conflicting_node = *conflicting_node_it;
    conflicting_node->distance_to_top = std::max(conflicting_node->distance_to_top, p_node->distance_to_top);
    conflicting_node->support_roof_layers_below = std::max(conflicting_node->support_roof_layers_below, p_node->support_roof_layers_below);
}

ModelVolumes::ModelVolumes(const SliceDataStorage& storage, coord_t xy_distance, coord_t max_move,coord_t max_move_slow ,coord_t radius_sample_resolution,coord_t z_distance_bottom_layers,
		coord_t z_distance_top_layers,bool support_rests_on_model,bool avoid_support_blocker, bool use_legacy_tree_support,bool use_exponential_collision_resolution,coord_t exponential_threashold,double exponential_factor) :
		machine_border_ { calculateMachineBorderCollision(
				storage.getMachineBorder()) }, xy_distance_ { xy_distance }, max_move_ {max_move },max_move_slow {max_move_slow}, radius_sample_resolution_ { radius_sample_resolution },z_distance_bottom_layers{z_distance_bottom_layers},
				z_distance_top_layers{z_distance_top_layers},support_rests_on_model{support_rests_on_model},avoid_support_blocker{avoid_support_blocker},use_legacy_tree_support{use_legacy_tree_support},use_exponential_collision_resolution{use_exponential_collision_resolution},
				exponential_threashold{exponential_threashold},exponential_factor{exponential_factor}
				{
	for (std::size_t layer_idx = 0;layer_idx < storage.support.supportLayers.size(); ++layer_idx) {
		constexpr bool include_support = false;
		constexpr bool include_prime_tower = true;

		layer_outlines_.push_back(storage.getLayerOutlines(layer_idx, include_support,include_prime_tower));
		if(avoid_support_blocker)
			anti_overhang_.push_back(storage.support.supportLayers[layer_idx].anti_overhang);
	}
}

const Polygons& ModelVolumes::getCollision(coord_t radius,LayerIndex layer_idx) const {
	radius = ceilRadius(radius);
	RadiusLayerPair key { radius, layer_idx };
	const auto it = collision_cache_.find(key);
	if (it != collision_cache_.end()) {
		collision_cache_hit++;
		return it->second;
	} else {
		collision_cache_miss++;
		return calculateCollision(key);
	}
}

const Polygons& ModelVolumes::getAvoidance(coord_t radius, LayerIndex layer_idx,bool slow,bool to_model) const {

	radius = ceilRadius(radius);
	RadiusLayerPair key { radius, layer_idx };

	if(to_model){
		if(slow){
				const auto it = avoidance_cache_to_model_slow.find(key);
				if (it != avoidance_cache_to_model_slow.end()) {
					return it->second;
				} else {
					return calculateAvoidance(key); // todo change to slow or assert false => see precalculate
				}
			}
			const auto it = avoidance_cache_to_model.find(key);
			if (it != avoidance_cache_to_model.end()) {
				return it->second;
			} else {
				return calculateAvoidance(key);
			}
	}
	else{
		if(slow){
			const auto it = avoidance_cache_slow.find(key);
			if (it != avoidance_cache_slow.end()) {
				return it->second;
			} else {
				return calculateAvoidance(key); // todo change to slow or assert false => see precalculate
			}
		}
		const auto it = avoidance_cache_.find(key);
		if (it != avoidance_cache_.end()) {
			return it->second;
		} else {
			return calculateAvoidance(key);
		}
	}
}

const Polygons& ModelVolumes::getPlaceableAreas(coord_t radius,LayerIndex layer_idx) const {
	radius = ceilRadius(radius);
	RadiusLayerPair key { radius, layer_idx };
	const auto it = placeable_areas_cache.find(key);
	if (it != placeable_areas_cache.end()) {
		return it->second;
	} else {
		logError("Trying to access non existing placeable area data data at radius: %lld layer: %lld. Empty polygon returned!\n",radius,layer_idx);
		return placeable_areas_cache[key];
	}
}

const Polygons& ModelVolumes::getInternalModel(coord_t radius,
		LayerIndex layer_idx) const {
	radius = ceilRadius(radius);
	RadiusLayerPair key { radius, layer_idx };
	const auto it = internal_model_cache_.find(key);
	if (it != internal_model_cache_.end()) {
		return it->second;
	} else {
		return calculateInternalModel(key);
	}
}

coord_t ModelVolumes::ceilRadius(coord_t radius) const {
	coord_t result=exponential_threashold;
	if(radius>=exponential_threashold&&use_exponential_collision_resolution){
		while(result<radius){
			result=std::max(coord_t(result*exponential_factor),result+radius_sample_resolution_);
		}
		return result;
	}

	const auto remainder = radius % radius_sample_resolution_;
	const auto delta =
			remainder != 0 ? radius_sample_resolution_ - remainder : 0;

	return radius + delta;
}

const Polygons& ModelVolumes::calculateCollision(
		const RadiusLayerPair &key) const {
	const auto &radius = key.first;
	const auto &layer_idx = key.second;

	if(!use_legacy_tree_support)
		logError("Had to recalculate at Collision radius %lld at %lld. This means multiple assurances made by the precalculateAvoidance are now void. Result may be invalid!\n",key.first,key.second);

	auto collision_areas = machine_border_;
	if (layer_idx < static_cast<int>(layer_outlines_.size())) {
		collision_areas = collision_areas.unionPolygons(layer_outlines_[layer_idx]);
	}
	collision_areas = collision_areas.offset(xy_distance_ + radius,
			ClipperLib::JoinType::jtRound);
	const auto ret = collision_cache_.insert(
			{ key, std::move(collision_areas) });
	assert(ret.second);
	return ret.first->second;
}

const Polygons& ModelVolumes::calculateAvoidance(const RadiusLayerPair &key) const { // todo add slow support
	const auto &radius = key.first;
	const auto &layer_idx = key.second;
	if(!use_legacy_tree_support)
		logError("Had to recalculate Avoidance at radius %lld at %lld. This means multiple assurances made by the precalculateAvoidance are now void, different avoidances do not exist at this layer and radius. Result should be expected to be invalid!\n",key.first,key.second);

	if (layer_idx == 0) {
		avoidance_cache_[key] = getCollision(radius, 0);
		return avoidance_cache_[key];
	}

	// Avoidance for a given layer depends on all layers beneath it so could have very deep recursion depths if
	// called at high layer heights. We can limit the reqursion depth to N by checking if the if the layer N
	// below the current one exists and if not, forcing the calculation of that layer. This may cause another recursion
	// if the layer at 2N below the current one but we won't exceed our limit unless there are N*N uncalculated layers
	// below our current one.
	constexpr auto max_recursion_depth = 100;
	// Check if we would exceed the recursion limit by trying to process this layer
	if (layer_idx >= max_recursion_depth
			&& avoidance_cache_.find(
					{ radius, layer_idx - max_recursion_depth })
					== avoidance_cache_.end()) {
		// Force the calculation of the layer `max_recursion_depth` below our current one, ignoring the result.
		getAvoidance(radius, layer_idx - max_recursion_depth);
	}
	auto avoidance_areas = getAvoidance(radius, layer_idx - 1).offset(
			-max_move_).smooth(5);
	avoidance_areas = avoidance_areas.unionPolygons(
			getCollision(radius, layer_idx));
	const auto ret = avoidance_cache_.insert(
			{ key, std::move(avoidance_areas) });
	assert(ret.second);
	return ret.first->second;
}

//ensures offsets are only done in sizes with a max step size per offset while adding the collision offset after each step, this ensures that areas cannot glitch through walls defined by the collision when offseting to fast
Polygons safeOffset(Polygons& me,coord_t distance,ClipperLib::JoinType jt,coord_t max_safe_step_distance,const Polygons& collision,double miter_limit = 1.2){
	const size_t steps = std::abs (distance/max_safe_step_distance);
	assert(distance*max_safe_step_distance>=0);
	Polygons ret=me;

	for(size_t i=0;i<steps;i++){
		ret=ret.offset(max_safe_step_distance,jt,miter_limit).unionPolygons(collision);
	}
	ret= ret.offset(distance%max_safe_step_distance,jt,miter_limit);

	return ret.unionPolygons(collision);
}


void ModelVolumes::precalculateAvoidance(const size_t maxLayer,const coord_t branch_radius, const double scaleFactor,const coord_t min_max_radius,const double scale_foot) {

	size_t min_max_height =0;
	double progress_total =0;
	for(coord_t counter=0;branch_radius+branch_radius*counter*scale_foot<min_max_radius;counter++){
		min_max_height=counter;
	}

	if(layer_outlines_.size()==0)
		return;

	const size_t model_height= layer_outlines_.size()-1;
	std::function<size_t(coord_t,coord_t)> getReguiredHeight=[&](coord_t radius,coord_t radius_before) {
		size_t maxReqLayer=maxLayer;
		if(radius>branch_radius){
			if(scaleFactor>0){

				double layers_handled_by_rad_before=(radius_before-branch_radius)/(branch_radius*scaleFactor);
				maxReqLayer-=layers_handled_by_rad_before-2; // -2 to avoid off by one problems and i dont care about a few layers more
			}
			if(maxReqLayer>maxLayer) maxReqLayer=maxLayer;
		}
		if(maxReqLayer<min_max_height) maxReqLayer=min_max_height;
		if(maxReqLayer>model_height) maxReqLayer=model_height;

		return maxReqLayer;
	};
	std::vector<int> collisionRadii; // todo remove add to resolution steps
	collisionRadii.push_back(0);

	const coord_t maxRadius=std::max(ceilRadius( branch_radius * (maxLayer+1) * scaleFactor+branch_radius),ceilRadius(min_max_radius));
	for (coord_t radius=use_legacy_tree_support?0:radius_sample_resolution_;radius<= maxRadius;radius=ceilRadius(radius+1)){
		resolution_steps.push_back(radius);
		collisionRadii.push_back(radius);
	}

	std::vector<Polygons> prePlaceable(maxLayer+1);
	//as we dont have multiple radiis for the support blocker we can not parallelize it
	if(avoid_support_blocker)
	for(size_t i=0;i<getReguiredHeight(0,0)-1&&i<anti_overhang_.size();i++){
		if(!anti_overhang_[i].empty()){
			anti_overhang_[i].simplify(30, 10);
			anti_overhang_[i+1]=anti_overhang_[i+1].unionPolygons(anti_overhang_[i].offset(-max_move_, ClipperLib::jtRound)); // we create a anti_overhang avoidance => because of this we can union it with the collision later
		}

	}

	// calculate all required collisions
#pragma omp parallel for schedule(static,1) shared(collision_cache_,prePlaceable)
	for(long long unsigned int i=0;i<collisionRadii.size();i++){
		coord_t radius = collisionRadii[i];

		size_t maxReqLayer=getReguiredHeight(radius,i!=0?collisionRadii[i-1]:0);
		if(radius==0) maxReqLayer=model_height; // we need r=0 everywhere
		std::vector<std::pair<RadiusLayerPair,Polygons>> data(maxReqLayer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons())); // we need to set a invalid default element because sometines we seem to add one


		logDebug("precalculating collision: radius: %lld up to layer %lld\n",radius,maxReqLayer);
		RadiusLayerPair key(radius,0);

		for(size_t layer=0;layer<=maxReqLayer;layer++){
			key.second=layer;
			Polygons collision_areas = machine_border_;
			if (layer < layer_outlines_.size()) {
				collision_areas = collision_areas.unionPolygons(layer_outlines_[layer]);
			}
			collision_areas = collision_areas.offset(xy_distance_ + radius, ClipperLib::JoinType::jtRound);

			collision_areas.simplify(30,10);

			data[layer]=std::pair<RadiusLayerPair,Polygons>(key,collision_areas);

		}

		if(!use_legacy_tree_support){

			for(coord_t layer=static_cast<coord_t>(maxReqLayer);layer>=0;layer--){ // have to do signed as we are counting down
				for(coord_t i=1;i<=z_distance_bottom_layers&&layer-i>0;i++){
					data[layer].second=data[layer].second.unionPolygons(data[layer-i].second);
				}
				if(support_rests_on_model&& layer+1<static_cast<coord_t>(prePlaceable.size())&&layer<static_cast<coord_t>(maxReqLayer)&&radius==0){
					prePlaceable[layer+1]=data[layer].second.difference(data[layer+1].second);
				}
			}

			for(size_t layer=0;layer<=maxReqLayer;layer++){

				for(coord_t i=1;i<= z_distance_top_layers && i+layer <=maxReqLayer;i++){
					data[layer].second=data[layer].second.unionPolygons(data[layer+i].second);
				}
				//we add the anti overhang to the collision, as this anti_overhang was already converted into an anti_overhang avoidance we can just add it as we can always escape this avoidance with max move speed
				if(avoid_support_blocker&&!anti_overhang_[layer].empty())
					data[layer].second=data[layer].second.unionPolygons(anti_overhang_[layer].offset(radius, ClipperLib::JoinType::jtRound));// we add the support blocker to ensure there is no support inside the support blocker
			}
		}

		#pragma omp critical(collision_cache_)
		{
			collision_cache_.insert(data.begin(),data.end());
			const double progress_step = PROGRESS_PRECALC_COLL/collisionRadii.size();
			progress_total+=progress_step;
			Progress::messageProgress(Progress::Stage::SUPPORT, progress_total,PROGRESS_TOTAL);
		}

	}

// calculate avoidance
#pragma omp parallel shared(avoidance_cache_,avoidance_cache_slow)
	{
	#pragma omp for schedule(static,1) nowait // to ensure all cores get equal amount of stuff to calculate schedule guided ? dynamic mit hoher blocksize TODO
	for(long long unsigned int i=0;i<resolution_steps.size();i++){

		coord_t radius = resolution_steps[i];

		const coord_t max_step_move = 2*(radius+ xy_distance_);

		RadiusLayerPair key(radius,0);
		size_t maxReqLayer=getReguiredHeight(radius,i!=0?resolution_steps[i-1]:0);

		std::vector<std::pair<RadiusLayerPair,Polygons>> data(maxReqLayer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
		std::vector<std::pair<RadiusLayerPair,Polygons>> data_slow(maxReqLayer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));

		Polygons LastLayer=getCollision(radius,0);
		Polygons LastLayer_slow=getCollision(radius,0);
		LastLayer.simplify(30,10);
		LastLayer_slow.simplify(30,10);
		data[0]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer);
		data_slow[0]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer_slow);

		logDebug("precalculating regular avoidance: radius: %lld up to layer %lld\n",radius,maxReqLayer);

		for(size_t layer=1;layer<=maxReqLayer;layer++){
			key.second=layer;
			Polygons col=getCollision(radius,layer);

			LastLayer=safeOffset(LastLayer,-max_move_,ClipperLib::jtRound,-max_step_move,col).smooth(5);
			LastLayer.simplify(30,10);
			data[layer]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer);

			if(!use_legacy_tree_support){
				LastLayer_slow=safeOffset(LastLayer_slow,-max_move_slow,ClipperLib::jtRound,-max_step_move,col).smooth(5);
				LastLayer_slow.simplify(30,10);
				data_slow[layer]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer_slow);
			}

		}
		#pragma omp critical(avoidance_cache_)
		{
			avoidance_cache_.insert(data.begin(),data.end());
			avoidance_cache_slow.insert(data_slow.begin(),data_slow.end());
			const double progress_avo =(PROGRESS_PRECALC_AVO/resolution_steps.size())*(support_rests_on_model?0.5:1);
			progress_total+=progress_avo;
			Progress::messageProgress(Progress::Stage::SUPPORT, progress_total,PROGRESS_TOTAL);
		}

	}


	// calculate avoidance to model
	if(support_rests_on_model&&!use_legacy_tree_support){

	#pragma omp for schedule(static,1) // to ensure all cores get equal amount of stuff to calculate schedule guided ? dynamic mit hoher blocksize TODO
		for(long long unsigned int i=0;i<resolution_steps.size();i++){

			coord_t radius = resolution_steps[i];

			const coord_t max_step_move = 2*(radius+ xy_distance_);

			RadiusLayerPair key(radius,0);
			size_t maxReqLayer=getReguiredHeight(radius,i!=0?resolution_steps[i-1]:0);

			std::vector<std::pair<RadiusLayerPair,Polygons>> data(maxReqLayer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			std::vector<std::pair<RadiusLayerPair,Polygons>> data_slow(maxReqLayer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			std::vector<std::pair<RadiusLayerPair,Polygons>> data_placeable(maxReqLayer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			Polygons LastLayer=getCollision(radius,0);
			Polygons LastLayer_slow=getCollision(radius,0);
			LastLayer.simplify(30,10);
			LastLayer_slow.simplify(30,10);
			data[0]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer);
			data_slow[0]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer_slow);

			logDebug("precalculating avoidance to model: radius: %lld up to layer %lld\n",radius,maxReqLayer);

			for(size_t layer=1;layer<=maxReqLayer;layer++){
				//printf("precalc: avoidance layer %d vertexes: %d\n",layer,LastLayer.pointCount());
				key.second=layer;
				Polygons placeable=prePlaceable[layer].offset(-radius,ClipperLib::jtRound);

				Polygons col=getCollision(radius,layer);

				LastLayer=safeOffset(LastLayer,-max_move_,ClipperLib::jtRound,-max_step_move,col).difference(placeable).smooth(5);
				LastLayer_slow=safeOffset(LastLayer_slow,-max_move_slow,ClipperLib::jtRound,-max_step_move,col).difference(placeable).smooth(5);// removed .unionPolygons(getCollision(radius, layer))

				LastLayer.simplify(30,10);
				LastLayer_slow.simplify(30,10);
				data[layer]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer);
				data_slow[layer]=std::pair<RadiusLayerPair,Polygons>(key,LastLayer_slow);
				data_placeable[layer]=std::pair<RadiusLayerPair,Polygons>(key,placeable);
			}
			#pragma omp critical(avoidance_cache_)
			{
				avoidance_cache_to_model.insert(data.begin(),data.end());
				avoidance_cache_to_model_slow.insert(data_slow.begin(),data_slow.end());
				placeable_areas_cache.insert(data_placeable.begin(),data_placeable.end());
				const double progress_avo =(PROGRESS_PRECALC_AVO/resolution_steps.size())*0.5;
				progress_total+=progress_avo;
				Progress::messageProgress(Progress::Stage::SUPPORT, progress_total,PROGRESS_TOTAL);
			}

		}
	}

	}
}

const Polygons& ModelVolumes::calculateInternalModel(
		const RadiusLayerPair &key) const {
	const auto &radius = key.first;
	const auto &layer_idx = key.second;

	const auto &internal_areas = getAvoidance(radius, layer_idx).difference(
			getCollision(radius, layer_idx));
	const auto ret = internal_model_cache_.insert( { key, internal_areas });
	assert(ret.second);
	return ret.first->second;

}

Polygons ModelVolumes::calculateMachineBorderCollision(Polygon machine_border) {
	Polygons machine_volume_border;
	machine_volume_border.add(machine_border.offset(1000000)); //Put a border of 1m around the print volume so that we don't collide.
	machine_border.reverse(); //Makes the polygon negative so that we subtract the actual volume from the collision area.
	machine_volume_border.add(machine_border);
	return machine_volume_border;
}

} //namespace cura
