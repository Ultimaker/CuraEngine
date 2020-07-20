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
#define PROGRESS_PRECALC_COLL 1000
#define PROGRESS_PRECALC_AVO 6000
#define PROGRESS_GENERATE_NODES 1000
#define PROGRESS_AREA_CALC 2000
#define PROGRESS_TOTAL 10000

#define SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL false
#define SUPPORT_TREE_AVOID_SUPPORT_BLOCKER true
#define SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION true
#define SUPPORT_TREE_EXPONENTIAL_THREASHOLD 1000
#define SUPPORT_TREE_EXPONENTIAL_FACTOR 1.25
#define SUPPORT_TREE_COLLISION_RESOLUTION 250

namespace cura {



TreeSupport::TreeSupport(const SliceDataStorage &storage) {

	Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	const coord_t radius_sample_resolution =SUPPORT_TREE_COLLISION_RESOLUTION;
	config=TreeSupportSettings(mesh_group_settings);
	const size_t z_distance_bottom_layers = round_up_divide(mesh_group_settings.get<coord_t>("support_bottom_distance"),config.layer_height);
	coord_t z_distance_top_layers= round_up_divide(mesh_group_settings.get<coord_t>("support_top_distance"),config.layer_height);
	precalculated_support_layers=std::vector<Polygons>(storage.support.supportLayers.size());
	const bool avoid_support_blocker=SUPPORT_TREE_AVOID_SUPPORT_BLOCKER;
	const bool use_exponential_collission_resolution=SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION;
	const double exponential_factor=SUPPORT_TREE_EXPONENTIAL_FACTOR;
	const coord_t exponential_threashold=SUPPORT_TREE_EXPONENTIAL_THREASHOLD;

	volumes_ = ModelVolumes(storage, config.xy_distance,  std::max(config.maximum_move_distance-5,coord_t(0)), std::max(config.maximum_move_distance_slow-5,coord_t(0)), // -5 is to ensure movement is always a bit faster than the avoidance
			radius_sample_resolution, z_distance_bottom_layers, z_distance_top_layers, config.support_rests_on_model,avoid_support_blocker,
			use_exponential_collission_resolution, exponential_threashold,exponential_factor);
}


void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    const bool global_use_tree_support = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("support_tree_enable");

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

	std::vector<std::map<SupportElement*,Polygons*>> move_bounds(storage.support.supportLayers.size()); // value is the area where support may be placed. As this is calculated in CreateLayerPathing it is saved and reused in drawAreas

	for (SliceMeshStorage &mesh : storage.meshes) {
		if (mesh.settings.get<bool>("support_tree_enable")) {
			generateInitalAreas(mesh,move_bounds,storage);
		}
	}

	auto t_gen = std::chrono::high_resolution_clock::now();
	//Expand Influnce areas down.
	createLayerPathing(move_bounds);
	auto t_path = std::chrono::high_resolution_clock::now();
	//Set a point in each influence area
	createNodesFromArea(move_bounds);
	auto t_place = std::chrono::high_resolution_clock::now();
	//draw these points as circles
	drawAreas(move_bounds, storage);
	auto t_draw = std::chrono::high_resolution_clock::now();
	auto dur_pre_gen = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_precalc-t_start ).count();
	auto dur_gen = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_gen - t_precalc ).count();
	auto dur_path = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_path - t_gen ).count();
	auto dur_place =0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_place - t_path ).count();
	auto dur_draw = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_draw - t_place ).count();

	log("Calculating Avoidance: %.3lf Creating inital influence areas: %.3lf Influence area creation: %.3lf ms Placement of Points in InfluenceAreas: %.3lf ms Drawing result as support %.3lf ms\n",dur_pre_gen,dur_gen,dur_path,dur_place,dur_draw);

	for (auto &layer : move_bounds) {
		for (auto elem : layer){
			delete elem.second;
			delete elem.first->area;
			delete elem.first;
		}
	}
	move_bounds.clear();


    storage.support.generated = true;
}

coord_t TreeSupport::precalculate(SliceDataStorage &storage){

	size_t max_layer=0;
	for (SliceMeshStorage &mesh : storage.meshes) {
		const coord_t layer_height = mesh.settings.get<coord_t>("layer_height");
		const coord_t z_distance_top = mesh.settings.get<coord_t>(
				"support_top_distance");
		const size_t z_distance_top_layers = round_up_divide(z_distance_top,
				layer_height) + 1; //Support must always be 1 layer below overhang.
		if(mesh.overhang_areas.size()<=z_distance_top_layers)
		{
			continue;
		}
		for (size_t layer_nr = (mesh.overhang_areas.size() - z_distance_top_layers)-1;layer_nr!=0;layer_nr--) { // look for max relevant layer
				const Polygons &overhang = mesh.overhang_areas[layer_nr+ z_distance_top_layers];
				if (!overhang.empty()) {
					if(layer_nr>max_layer)// iterates over multiple meshes
					{
						max_layer=1+layer_nr; // plus one to avoid problems if something is of by one
					}
					break;
				}
			}
	}

	volumes_.precalculateAvoidance(max_layer, config.branch_radius, config.diameter_angle_scale_factor,config.bp_radius,config.diameter_scale_elephant_foot);
	return max_layer;
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
		std::unordered_map<SupportElement,Polygons>& insert_main,
		std::unordered_map<SupportElement,Polygons>& insert_secondary,
		std::vector<SupportElement>& erase,
		coord_t layer_nr )const{

	for(std::map<SupportElement,Polygons>::iterator influence_iter = input.begin(); influence_iter != input.end(); influence_iter++){
			bool merged=false;

			AABB influence_aabb = input_aabb.count(influence_iter->first)?input_aabb.at(influence_iter->first):AABB(influence_iter->second);
			for(std::map<SupportElement,AABB>::iterator reduced_check_iter = reduced_new_layer_aabb.begin(); reduced_check_iter != reduced_new_layer_aabb.end(); reduced_check_iter++){

				AABB aabb = reduced_check_iter->second;
				if(aabb.hit(influence_aabb)) {

					bool merging_gracious_and_non_gracious = reduced_check_iter->first.to_model_gracious != influence_iter->first.to_model_gracious; // we do not want to merge a gracious with a non gracious area as bad placement could kill the hole subtree
					bool merging_to_bp=reduced_check_iter->first.to_buildplate&&influence_iter->first.to_buildplate; // get from main otherwise get from secondary

					size_t increased_dtt=0;
					size_t smaller_to_model_ddt=0;

					if(!merging_to_bp){

						coord_t eff_dtt_infl=config.getEffektiveDDT(influence_iter->first);
						coord_t eff_ddt_redu=config.getEffektiveDDT(reduced_check_iter->first);
						if(reduced_check_iter->first.to_buildplate!=influence_iter->first.to_buildplate){ // calculate increased DDT and total_to_model_ddt
							if(reduced_check_iter->first.to_buildplate){
								if(eff_dtt_infl<eff_ddt_redu){
									increased_dtt=influence_iter->first.increased_ddt + reduced_check_iter->first.distance_to_top-influence_iter->first.distance_to_top;
								}
							}
							else{
								if(eff_dtt_infl>eff_ddt_redu){
									increased_dtt=reduced_check_iter->first.increased_ddt + influence_iter->first.distance_to_top-reduced_check_iter->first.distance_to_top;
								}
							}
						}
						smaller_to_model_ddt=eff_dtt_infl<eff_ddt_redu ? eff_dtt_infl:eff_ddt_redu;

					}

					// if we could place ourself on unstable ground, would be increasing our radius further than we are allowed to when merging to model and to bp trees or we merge 2 non gracious before we know we will even draw this subtree we dont merge
					// also we check if one of both has moved, if no we wont have anything to merge
					if(merging_gracious_and_non_gracious||increased_dtt>config.max_ddt_increase||(smaller_to_model_ddt<config.min_ddt_to_model&& !reduced_check_iter->first.to_model_gracious && !influence_iter->first.to_model_gracious)){
						continue;
					}

					Polygons relevant_infl;
					Polygons relevant_redu;

					if(merging_to_bp)
					{
						relevant_infl= main.count(influence_iter->first)?main.at(influence_iter->first):Polygons(); // is a new element dont need to check if i changed it, cause i didnt
						relevant_redu= insert_main.count(reduced_check_iter->first)?insert_main[reduced_check_iter->first]:(main.count(reduced_check_iter->first)?main.at(reduced_check_iter->first):Polygons());
					}
					else{
						relevant_infl= secondary.count(influence_iter->first)?secondary.at(influence_iter->first):Polygons();
						relevant_redu= insert_secondary.count(influence_iter->first)?insert_secondary[reduced_check_iter->first]:(secondary.count(reduced_check_iter->first)?secondary.at(reduced_check_iter->first):Polygons());
					}

					const bool red_bigger=config.getEffektiveDDT(reduced_check_iter->first)>config.getEffektiveDDT(influence_iter->first);
					std::pair<SupportElement,Polygons> smaller_rad = red_bigger ? std::pair<SupportElement,Polygons>(influence_iter->first,relevant_infl) : std::pair<SupportElement,Polygons>(reduced_check_iter->first,relevant_redu);
					std::pair<SupportElement,Polygons> bigger_rad =red_bigger ? std::pair<SupportElement,Polygons>(reduced_check_iter->first,relevant_redu) : std::pair<SupportElement,Polygons>(influence_iter->first,relevant_infl);
					const coord_t big_radius=config.getRadius(bigger_rad.first);
					const coord_t small_radius=config.getRadius(smaller_rad.first);
					Polygons small_rad_increased_by_big_minus_small;
					Polygons intersect;
					if(config.getCollisionRadius(bigger_rad.first)<config.getCollisionRadius(smaller_rad.first)){
						small_rad_increased_by_big_minus_small=bigger_rad.second.offset(big_radius-small_radius, ClipperLib::jtRound);
						intersect= small_rad_increased_by_big_minus_small.intersection(smaller_rad.second);
					}
					else{
						small_rad_increased_by_big_minus_small=smaller_rad.second.offset(big_radius-small_radius, ClipperLib::jtRound);
						intersect= small_rad_increased_by_big_minus_small.intersection(bigger_rad.second);
					}

					if(intersect.area()>1){ // dont use empty as a line is not empty, but for our usecase it very well may be (and will be one layer down, union does not keep lines)

						if(intersect.offset(-25).area()<=1) // check if the overlap we have is good enough. While 25 was guessed as enough, until now i did not have reason to doubt that
							continue;

						Point new_pos = reduced_check_iter->first.next_position;
						if(!intersect.inside(new_pos, true)){
							PolygonUtils::moveInside(intersect, new_pos);
						}

						if(increased_dtt==0){
							increased_dtt=std::max(reduced_check_iter->first.increased_ddt,influence_iter->first.increased_ddt);
						}

						SupportElement key(reduced_check_iter->first,influence_iter->first,layer_nr-1,new_pos,increased_dtt,config);

						Polygons intersect_sec;
						if(merging_to_bp&&config.support_rests_on_model){

							Polygons secSmall=insert_secondary.count(smaller_rad.first)?insert_secondary[smaller_rad.first]:(secondary.count(smaller_rad.first)?secondary.at(smaller_rad.first):Polygons());
							Polygons secBig=insert_secondary.count(bigger_rad.first)?insert_secondary[bigger_rad.first]:(secondary.count(bigger_rad.first)?secondary.at(bigger_rad.first):Polygons());
							Polygons small_rad_increased_by_big_minus_small_sec;
							if(config.getCollisionRadius(bigger_rad.first)<config.getCollisionRadius(smaller_rad.first)){
								small_rad_increased_by_big_minus_small_sec=secBig.offset(big_radius-small_radius, ClipperLib::jtRound);
								intersect_sec= small_rad_increased_by_big_minus_small_sec.intersection(secSmall);// if the one with the bigger radius with the lower radius removed overlaps we can merge
							}
							else{

								small_rad_increased_by_big_minus_small_sec=secSmall.offset(big_radius-small_radius, ClipperLib::jtRound);
								intersect_sec= small_rad_increased_by_big_minus_small_sec.intersection(secBig);// if the one with the bigger radius with the lower radius removed overlaps we can merge
							}

						}

						insert_main.erase(reduced_check_iter->first);
						insert_main.erase(influence_iter->first);
						insert_secondary.erase(reduced_check_iter->first);
						insert_secondary.erase(influence_iter->first);

						(merging_to_bp?insert_main:insert_secondary).emplace(key,intersect);
						if(merging_to_bp&&config.support_rests_on_model)
							insert_secondary.emplace(key,intersect_sec);
						erase.emplace_back(reduced_check_iter->first);
						erase.emplace_back(influence_iter->first);
						Polygons merge = intersect.unionPolygons(intersect_sec).offset(config.getRadius(key), ClipperLib::jtRound).difference(volumes_.getCollision(0, layer_nr-1)); //we dont need to safe offset here as it should already be correctly of the intersect area being away enough

						reduced_new_layer.erase(reduced_check_iter->first);
						reduced_new_layer.emplace(key,merge);

						reduced_new_layer_aabb.erase(reduced_check_iter->first); // this invalidates reduced_check_iter
						reduced_new_layer_aabb.emplace(key,AABB(merge));

						merged=true;
						break;
					}
				}
			}

			if(!merged){
				reduced_new_layer[influence_iter->first]=influence_iter->second;
				reduced_new_layer_aabb[influence_iter->first]=influence_aabb;
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

		std::vector<std::unordered_map<SupportElement,Polygons>> insert_main(parts.size()/2);
		std::vector<std::unordered_map<SupportElement,Polygons>> insert_secondary(parts.size()/2);
		std::vector<std::vector<SupportElement>> erase(parts.size()/2);

		#pragma omp parallel for schedule(dynamic)
		for(coord_t i=0;i<(coord_t)parts.size()-1;i=i+2){

			mergeHelper(parts[i], parts_aabb[i], parts[i+1],parts_aabb[i+1], main, secondary, insert_main[i/2], insert_secondary[i/2], erase[i/2], layer_nr);
			parts[i+1].clear();
			parts_aabb[i+1].clear();

		}

		for(coord_t i=0;i<(coord_t)parts.size()-1;i=i+2){

			for(SupportElement& del : erase[i/2]){
				main.erase(del);
				secondary.erase(del);
			}

			for(const std::pair<SupportElement,Polygons>& tup : insert_main[i/2]){
				main.emplace(tup);
			}

			for(const std::pair<SupportElement,Polygons>& tup : insert_secondary[i/2]){
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

Polygons TreeSupport::safeOffsetInc(Polygons& me,coord_t distance,const Polygons& collision,coord_t last_safe_step_size,size_t min_amount_offset)const{

	if(distance==0)
		return me.difference(collision).unionPolygons();

	size_t counted=0;
	size_t steps = distance>last_safe_step_size ? std::abs ((distance-last_safe_step_size)/config.xy_distance):0;
	size_t step_size=config.xy_distance;
	if(steps+(distance<last_safe_step_size||distance%step_size!=0)<min_amount_offset ){ // yes we can add a bool as the standard specifies that a result from compare operators has to be 0 or 1
		// we need to reduce the stepsize to ensure we offset the required amount (could be avoided if arcRadiance were exposed as we could just increase this when me has not enough vertices)
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
void TreeSupport::increaseAreas(std::unordered_map<SupportElement,Polygons>& new_layer,std::map<SupportElement,Polygons>& new_layer_merge,std::unordered_map<SupportElement,Polygons>& new_layer_to_model,std::vector<std::pair<SupportElement*,Polygons*>> last_layer,size_t layer_nr){

#pragma omp parallel for shared(new_layer,new_layer_merge,new_layer_to_model,last_layer)
		for(long long unsigned int i=0;i<last_layer.size();i++){

			std::pair<SupportElement*,Polygons*> tup = last_layer[i];

			SupportElement elem(*tup.first,tup.first); // also increases ddt

			coord_t extra_over_speed=0;

			// we assume that the good areas from our new collision will overlap with the good areas of our old collision, but we have to use 0 for our old collision as we may not fit a whole circle in there in some cases i think
			// this may not be the most efficient way as a bigger collision would increase our safeStepSize, but it is a correct one
			const Polygons wall_restriction=volumes_.getCollision(0, layer_nr).intersection(volumes_.getCollision(config.getCollisionRadius(*tup.first), layer_nr-1)); // second also 0 rad ?

			//Polygons newLayerDataSlow=tup.first->area->safeOffset(config.maximum_move_distance_slow,ClipperLib::jtRound,max_step_move,wallRestriction,true); // we need to offset in 2 steps as we loose areas if we only inc small areas once sometimes
			Polygons new_layer_data,new_layer_data_to_model,check_layer_data,increased;
			size_t radius;
			std::function<bool(bool,bool,bool)> setPolygons=[&](bool slow,bool increase_radius,bool simplify) { // this correctly sets the polygons defined above according to the parameters and returns if a new layer is possible
				SupportElement simulate_increase(elem);
				if(increase_radius)
					simulate_increase.effective_radius_height+=1;
				radius = increase_radius? config.getCollisionRadius(simulate_increase) : config.getCollisionRadius(elem);

				coord_t extra_move_speed= config.getRadius(simulate_increase)-config.getRadius(*tup.first); //todo limit ?
				if(extra_move_speed<0)
					logWarning("Tree Support: extraMoveSpeed<0. This implies that a branch is getting smaller while going down. This may cause problems.");
				extra_move_speed+=extra_over_speed;
				increased=safeOffsetInc(*tup.first->area, extra_move_speed + (slow?config.maximum_move_distance_slow:config.maximum_move_distance), wall_restriction, 2*(config.xy_distance+config.getCollisionRadius(*tup.first)), 2); // offsetting in 2 steps makes our offseted area rounder preventing (rounding) errors created by to pointy areas. May not be a problem anymore though.

				if(simplify){
					increased=increased.smooth(5);
					increased.simplify(15);
				}
				new_layer_data= increased.difference(volumes_.getAvoidance(radius,layer_nr-1,slow)).unionPolygons();
				if(config.support_rests_on_model){
					 new_layer_data_to_model= increased.difference(volumes_.getAvoidance(radius, layer_nr-1,slow , true)).unionPolygons();
					 if(!elem.to_model_gracious){
						 if(new_layer_data_to_model.area()>=1){
							 elem.to_model_gracious=true;
							 logWarning("Corrected to model taint on layer %lld targeting %lld with radius %lld\n",layer_nr-1,elem.target_height,radius);

						 }
						 else{
							 new_layer_data_to_model= increased.difference(volumes_.getCollision(radius, layer_nr-1)).unionPolygons();
						 }
					 }
				}
				if(new_layer_data.area()>1&& !elem.to_buildplate){ //mostly happening in the tip, but with merges ...
					elem.to_buildplate=true; // sometimes nodes that can reach the buildplate are marked as cant reach, taining subtrees. this fixes this
					logWarning("Corrected to buildplate taint on layer %lld targeting %lld with radius %lld\n",layer_nr-1,elem.target_height,radius);
				}
				check_layer_data=elem.to_buildplate ? new_layer_data : new_layer_data_to_model;



				if(increase_radius&&check_layer_data.area()>1){
					std::function<bool(bool,bool,bool)> nextStep=[&](bool slow_2,bool increase_radius_2,bool simplify_2) { // saves current status, and trys another step with the same settings to check if the change was valid
						Polygons new_layer_data_backup=Polygons(new_layer_data); // remember, setPolygons is by DEFINITION not non destructive
						Polygons check_layer_data_backup=Polygons(check_layer_data);
						Polygons new_layer_data_to_model_backup=Polygons(new_layer_data_to_model);
						size_t radius_backup=radius;

						if(!setPolygons(slow_2,increase_radius_2,simplify_2)){ // we can not catch up further
							check_layer_data=check_layer_data_backup;
							new_layer_data=new_layer_data_backup;
							new_layer_data_to_model=new_layer_data_to_model_backup;
							radius=radius_backup;
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
						else{
							radius=config.getCollisionRadius(elem);
						}
					}

					if(config.getRadius(elem) < config.increase_radius_until_radius&&elem.effective_radius_height<elem.distance_to_top){

						coord_t orig_ceil= volumes_.ceilRadius(config.getRadius(elem.effective_radius_height,elem.elephant_foot_increases));
						bool increasing_radius=config.getRadius(elem.effective_radius_height)!=config.getRadius(elem.effective_radius_height+1);
						size_t max_effective_dtt = elem.effective_radius_height;
						for (coord_t i =1;volumes_.ceilRadius(config.getRadius(elem.effective_radius_height+i,elem.elephant_foot_increases))==orig_ceil&&increasing_radius;i++){ // todo make it mathematically nice, instead of simulating radius increases

							if(config.getRadius(elem.effective_radius_height+i)==config.getRadius(elem.effective_radius_height+i+1))// we reached a point where we can not increase our radius further
							{
								max_effective_dtt=std::numeric_limits<coord_t>::max(); // yeah i know this is missing one bit, i just want to make sure i dont accidently overlow
								break;
							}
							max_effective_dtt++;
						}
						elem.effective_radius_height=max_effective_dtt>config.getEffektiveDDT(elem)? config.getEffektiveDDT(elem):max_effective_dtt;
						radius=config.getCollisionRadius(elem);
						bool another_step =max_effective_dtt<config.getEffektiveDDT(elem);

						if(another_step){
							nextStep(slow,true,simplify);
						}
					}
				}

				return check_layer_data.area()>1;
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

				logError("Influence area could not be increased! Data about the Influence area: Radius: %lld at layer: %lld NextTarget: %lld Distance to top: %lld Elephant foot increases %f \n",radius,layer_nr-1,elem.next_height,elem.distance_to_top,elem.elephant_foot_increases);
				extra_over_speed=config.maximum_move_distance/2;
				if(setPolygons(false,false,false)){
					logError("Trying to keep area by moving faster than intended: Success \n");
					add=true;
				}
				else{
					logError("Trying to keep area by moving faster than intended: FAILURE! WRONG BRANCHES LIKLY! \n");
				}
			}

			if(add){

				Polygons merge; // we should make sure that the outer wall is correct

				Polygons merge_area_inc=(config.support_rests_on_model?new_layer_data_to_model:new_layer_data).offset(config.getRadius(elem), ClipperLib::jtRound);
				merge = merge_area_inc.difference(volumes_.getCollision(0, layer_nr-1));

				#pragma omp critical(newLayer)
				{
					new_layer_merge.emplace(elem,merge);
					if(elem.to_buildplate){
						new_layer.emplace(elem,new_layer_data);
					}
					if(config.support_rests_on_model){
						new_layer_to_model.emplace(elem,new_layer_data_to_model);
					}
				}
			}

		}

}


void TreeSupport::createLayerPathing(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds){

	const double data_size_inverse = 1/double(move_bounds.size());
	double progress_total =PROGRESS_PRECALC_AVO+PROGRESS_PRECALC_COLL+PROGRESS_GENERATE_NODES;

	auto dur_inc=std::chrono::duration_values<std::chrono::nanoseconds>::zero();
	auto dur_merge=std::chrono::duration_values<std::chrono::nanoseconds>::zero();

	//Increase Influence Radius of each influence circle check for overlap and insert a new influence circle
	for (size_t layer_nr = move_bounds.size() - 1; layer_nr > 0; layer_nr--){

		std::map<SupportElement,Polygons> new_layer_merge; // merge maps are increased by effective radius to be able to ensure correct merge when 2 elements have a different radius
		std::unordered_map<SupportElement,Polygons> new_layer,new_layer_to_model;
		auto ta = std::chrono::high_resolution_clock::now();

		std::vector<std::pair<SupportElement*,Polygons*>> last_layer;
		last_layer.insert(last_layer.begin(), move_bounds[layer_nr].begin(), move_bounds[layer_nr].end());

		increaseAreas(new_layer, new_layer_merge, new_layer_to_model, last_layer, layer_nr);

		auto tb = std::chrono::high_resolution_clock::now();

		mergePolygonsDaQ(new_layer_merge, new_layer, new_layer_to_model, layer_nr);


		auto tc = std::chrono::high_resolution_clock::now();

		dur_inc+=tb-ta;
		dur_merge+=tc-tb;


		for(std::pair<SupportElement,Polygons> tup:new_layer){


			const SupportElement elem = tup.first;
			Polygons* new_area= new Polygons(tup.second.unionPolygons(new_layer_to_model.count(elem)?new_layer_to_model[elem]:Polygons()));
			SupportElement* next= new SupportElement(elem,new_area);
			move_bounds[layer_nr-1][next]=new Polygons(new_layer_merge[tup.first]);

			if(new_area->area()<1){
				logError("Insert Error of Influence area to buildplate on layer %lld.\n",layer_nr-1);
			}
		}

		for(std::pair<SupportElement,Polygons> tup:new_layer_to_model){
			const SupportElement& elem = tup.first;

			if(elem.to_buildplate) continue;
			Polygons* new_area= new Polygons(tup.second);
			SupportElement* next= new SupportElement(elem,new_area);
			move_bounds[layer_nr-1][next]=new Polygons(new_layer_merge[tup.first]);

			if(new_area->area()<1)
				logError("Insert Error of Influence area to model on layer %lld.\n",layer_nr-1);

		}

		progress_total+=data_size_inverse * PROGRESS_AREA_CALC;
		Progress::messageProgress(Progress::Stage::SUPPORT, progress_total,PROGRESS_TOTAL);

	}

    log("Total time increasing influence areas: %lld ms Total time merging influence areas: %lld ms\n",dur_inc.count()/1000000,dur_merge.count()/1000000);


}


void TreeSupport::setPointsOnAreas(SupportElement* elem, coord_t max_move){

	if(elem->result_on_layer==Point(-1,-1)){
		logError("Uninitialised support element\n");
		return;
	}

	for(SupportElement* next_elem : elem->parents){

		if(next_elem->result_on_layer!=Point(-1,-1)) // if we set the value somewhere else we keep it; should only happen at the top most layer
			continue;

		Point from = elem->result_on_layer;
		if(!(next_elem->area->inside(from, true))){ // todo else try move to next position
			PolygonUtils::moveInside(*next_elem->area,from,0); // Move inside has edgecases (see tests) so DONT use Polygons.inside, Error with dist 0 is <= 1 //
		}
		if(vSize(from-elem->result_on_layer)>max_move){ // while this SEEMS like a problem it may occur after merges or because the radius changed => which is free movement speed

		}
		next_elem->result_on_layer=from;
		//no we do not call recursive because if we do out amount of layers is restricted by our stack size -.-'
	}

}

bool TreeSupport::setToModelContact (std::vector<std::map<SupportElement*,Polygons*>>& move_bounds, SupportElement* first_elem ,const size_t layer_nr){


	if(first_elem->to_model_gracious){

		SupportElement* check=first_elem;

		std::vector<SupportElement*> checked({check});
		size_t last_successfull_layer=layer_nr;

		for(size_t layer_check=layer_nr;check->next_height<layer_check;layer_check++){

			if(!check->area->intersection(volumes_.getPlaceableAreas(config.getRadius(*check), layer_check)).empty()){
				last_successfull_layer=layer_check;
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


		for(size_t layer=layer_nr+1;layer<last_successfull_layer-1;layer++){
			move_bounds[layer].erase(checked[layer-layer_nr]);
			delete checked[layer-layer_nr]->area;
			delete checked[layer-layer_nr];
		}

		// set Pos
		Point best=checked[last_successfull_layer-layer_nr]->next_position;
		if(!checked[last_successfull_layer-layer_nr]->area->inside(best, true))
			PolygonUtils::moveInside(*checked[last_successfull_layer-layer_nr]->area,best);
		checked[last_successfull_layer-layer_nr]->result_on_layer=best;

		logDebug("Added gracious Support On Model on layer Point (%lld,%lld) on current layer is %lld\n",best.X,best.Y,last_successfull_layer);

		return last_successfull_layer!=layer_nr;


	}
	else{ // can not add gracefull => just place it here and hope for the best
		Point best=first_elem->next_position;
		if(!first_elem->area->inside(best, true))
			PolygonUtils::moveInside(*first_elem->area,best);
		first_elem->result_on_layer=best;
		logDebug("added NON gracious  Support On Model on layer Point (%lld,%lld) my current layer is %lld\n",best.X,best.Y,layer_nr);
		return false;
	}

}



void TreeSupport::createNodesFromArea(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds){

	const coord_t maximum_move_distance =config.maximum_move_distance + 3; // movement needs to be a bit higher to avoid rounding errors

	for(std::pair<SupportElement*,Polygons*> init : move_bounds[0]){ // init points on layer 0
		Point p=init.first->next_position;
		if(!(init.first->area->inside(p, true)))
			PolygonUtils::moveInside(*init.first->area,p,0 );
		init.first->result_on_layer=p;
		setPointsOnAreas(init.first, maximum_move_distance);
	}

	std::map<SupportElement,int> ignore;

	for (size_t layer_nr = 1; layer_nr < move_bounds.size(); layer_nr++){
		std::unordered_set<SupportElement*> remove;
		for(std::pair<SupportElement*,Polygons*> influence_area_tup : move_bounds[layer_nr]){

			bool removed=false;
			if(influence_area_tup.first->result_on_layer==Point(-1,-1)){
				if(influence_area_tup.first->to_buildplate){
					logError("UNKNOWN POLYGON targeting (%lld,%lld) at target_height: %lld layer: %lld\n",influence_area_tup.first->target_position.X,influence_area_tup.first->target_position.Y,influence_area_tup.first->target_height,layer_nr);
				}
				else{// we need to connect with the model

					if(config.getEffektiveDDT(*influence_area_tup.first)<config.min_ddt_to_model){
						remove.emplace(influence_area_tup.first); // we dont need to remove the parrents as they will have a lower ddt
						removed=true;
						for(SupportElement* elem:influence_area_tup.first->parents)
							elem->to_buildplate=false;
						continue;
					}

					removed=setToModelContact(move_bounds, influence_area_tup.first, layer_nr);
					if(removed)
						remove.emplace(influence_area_tup.first);
				}
			}

			if(!removed)
				setPointsOnAreas(influence_area_tup.first, maximum_move_distance);//element is valid now setting points in the layer above
		}

		for(SupportElement* del :remove){ // delete all support elements we dont need anymore
			delete move_bounds[layer_nr][del];
			move_bounds[layer_nr].erase(del);
			delete del->area;
			delete del;
		}
		remove.clear();
	}

}

void TreeSupport::drawAreas(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds,SliceDataStorage &storage){
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

	std::map<SupportElement*,std::pair<SupportElement*,Polygons*>> inverese_tree_order; // in our tree we can only access the partents. We inverse this to be able to access the children.
	std::vector<std::pair<size_t,std::pair<SupportElement*,Polygons*>>> linear_data; // we put all SupportElements into a layer independent storage to improve parallelization. Was added at a point in time where this function had performance issues.
																					// These were fixed by creating less initial points, but i do not see a good reason to remove a working performance optimization.
	for (size_t layer_nr = 0; layer_nr < move_bounds.size();layer_nr++) {
		for(std::pair<SupportElement*,Polygons*> tup: move_bounds[layer_nr]){

			if((layer_nr>0&&((!inverese_tree_order.count(tup.first)&&tup.first->target_height==layer_nr)||(inverese_tree_order.count(tup.first)&&inverese_tree_order[tup.first].first->result_on_layer==Point(-1,-1))))) // we either come from nowhere at the final layer or we had invalid parents 2. should never happen but just to be sure
				continue;

			for(SupportElement* par : tup.first->parents){
				if(par->result_on_layer==Point(-1,-1))
					continue;
				inverese_tree_order.emplace(par,tup);
			}
			linear_data.emplace_back(layer_nr,tup);
		}
	}

	std::vector<Polygons> linear_inserts(linear_data.size());
	// parallel iterating over all elements
#pragma omp parallel for shared(linear_inserts, linear_data, inverese_tree_order)
	for(coord_t i=0;i<static_cast<coord_t>(linear_data.size());i++){
		const std::pair<SupportElement*,Polygons*> tup= linear_data[i].second;

		Polygon circle;
		double scale = (1.0 + static_cast<double>(config.getEffektiveDDT(*tup.first) - tip_layers) * diameter_angle_scale_factor);
		if (config.getEffektiveDDT(*tup.first) < tip_layers){ //We're in the tip.
			scale = static_cast<double>(config.getEffektiveDDT(*tup.first) + 1) / tip_layers;
		}
		scale = config.getRadius(*tup.first)/(1.0*branch_radius);

		Point movement;
		SupportElement* child_elem =  inverese_tree_order.count(tup.first)?inverese_tree_order.at(tup.first).first:nullptr;
		if(child_elem!=nullptr){
			movement =(child_elem->result_on_layer - tup.first->result_on_layer);
			if(tup.first->next_height==linear_data[i].first&&movement.X+movement.Y){ // if merge limit move distance to avoid excessive ovalization
				coord_t radius_difference=  std::abs(config.getRadius(*tup.first)-config.getRadius(*child_elem));
				movement=movement*(vSize(movement)-radius_difference>0?vSize(movement)-radius_difference:0)/vSize(movement);
			}
		}
		else if(tup.first->parents.size()!=0){//only happens at the most bottom area of the support
			SupportElement* biggest_radius_parrent=getBiggestRadiusParrent(tup.first);
			movement =( biggest_radius_parrent->result_on_layer - tup.first->result_on_layer);
			if(biggest_radius_parrent->next_position==linear_data[i].first&&movement.X+movement.Y){ // if merge limit move distance to avoid excessive ovalization
				coord_t radiusDifference=  std::abs(config.getRadius(*tup.first)-config.getRadius(*biggest_radius_parrent));
				movement=movement*(vSize(movement)-radiusDifference>0?vSize(movement)-radiusDifference:0)/vSize(movement);
			}
		}
		else{
			logWarning("Ovalisation failed! This means a element has neither a child nor a parent!\n");
		}

		Point center_position = tup.first->result_on_layer+movement/2;

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
			circle.add(center_position + vertex);
		}


		Polygons poly;
		poly.add(circle);

		const Polygons child = inverese_tree_order.count(tup.first)?*inverese_tree_order.at(tup.first).second:Polygons();
		linear_inserts[i]=tup.second->unionPolygons(child).intersection(poly.unionPolygons());
	}

	std::vector<std::vector<Polygons>> byLayer(move_bounds.size());
	// single threaded combining all elements to the right layers. ONLY COPYS DATA!
	for(coord_t i=0;i<static_cast<coord_t>(linear_data.size());i++){
		precalculated_support_layers[linear_data[i].first].add(linear_inserts[i]);
	}


	linear_inserts.clear();
	linear_data.clear();

	//now we iterate over the inserted elements in parallel and clean them up
#pragma omp parallel for shared(precalculated_support_layers, storage)
	for (coord_t layer_nr = 0; layer_nr < static_cast<coord_t>(precalculated_support_layers.size());layer_nr++) {
			precalculated_support_layers[layer_nr] = precalculated_support_layers[layer_nr].unionPolygons().smooth(50);
			storage.support.supportLayers[layer_nr].support_roof = storage.support.supportLayers[layer_nr].support_roof.unionPolygons();
			precalculated_support_layers[layer_nr] = precalculated_support_layers[layer_nr].difference(storage.support.supportLayers[layer_nr].support_roof);
			precalculated_support_layers[layer_nr].simplify(30,10);
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
							precalculated_support_layers[layer_nr].intersection(
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
							precalculated_support_layers[layer_nr].intersection(
									storage.getLayerOutlines(sample_layer,
											no_support, no_prime_tower)));
				}
				floor_layer.unionPolygons();
				precalculated_support_layers[layer_nr] = precalculated_support_layers[layer_nr].difference(floor_layer.offset(10)); //Subtract the support floor from the normal support.
			}

		for (PolygonsPart part : precalculated_support_layers[layer_nr].splitIntoParts(false) ) //Convert every part into a PolygonsPart for the support.
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

	Polygons result_poly;

	for(std::deque<Point> line:result){

		Polygon result_line;
		for(Point p:line)
			result_line.add(p);
		result_poly.add(result_line);
	}
	return result_poly;

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
		LineInformation res_line;
		for(Point p:line){
			if(!fastInside(p,avoid_to_bp)){
				res_line.emplace_back(p,LineStatus::TO_BP);
			}
			else if(config.support_rests_on_model&&!fastInside(p,avoid_to_model)){
				res_line.emplace_back(p,LineStatus::TO_MODEL_GRACIOUS);
			}
			else if(config.support_rests_on_model&&!fastInside(p,collision)){
				res_line.emplace_back(p,LineStatus::TO_MODEL);
			}
			else{
				if(!res_line.empty()){
					result.emplace_back(res_line);
					res_line.clear();
				}
			}

		}
		if(!res_line.empty()){
			result.emplace_back(res_line);
			res_line.clear();
		}
	}

	return result;

}

Polygons TreeSupport::convertInternalToLines(std::vector<TreeSupport::LineInformation> lines){
	Polygons result;

	for(LineInformation line:lines){
		Polygon path;
		for(auto point_data:line){
			path.add(point_data.first);
		}
		result.add(path);
	}
	return result;
}


//splits the lines into lines that can be migrated down and lines that have to go into the tree algorithm
std::pair<std::vector<TreeSupport::LineInformation>,std::vector<TreeSupport::LineInformation>> TreeSupport::splitLines(std::vector<TreeSupport::LineInformation> lines,size_t current_layer,size_t ddt){//assumes all Points on the current line are valid

	std::vector<LineInformation> keep(1);
	std::vector<LineInformation> set_free(1);
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
		LineInformation resulting_line;
		for(std::pair<Point,LineStatus> me:line){

			if(evaluatePoint(me)){
				if(keeping!=current){
					if(!resulting_line.empty()){
						set_free.emplace_back(resulting_line);
						resulting_line.clear();
					}
					current=keeping;
				}
				resulting_line.emplace_back(me);
			}
			else{
				if(freeing!=current){
					if(!resulting_line.empty()){
						keep.emplace_back(resulting_line);
						resulting_line.clear();
					}
					current=freeing;
				}
				resulting_line.emplace_back(me);
			}

		}
		if(!resulting_line.empty()){
			if(current==keeping){
				keep.emplace_back(resulting_line);
			}
			else{
				set_free.emplace_back(resulting_line);
			}
		}
	}

	return std::pair<std::vector<std::vector<std::pair<Point,TreeSupport::LineStatus>>>,std::vector<std::vector<std::pair<Point,TreeSupport::LineStatus>>>> (keep,set_free);


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

void TreeSupport::generateInitalAreas(const SliceMeshStorage& mesh,std::vector<std::map<SupportElement*,Polygons*>>& move_bounds,SliceDataStorage& storage){

	Polygon base_circle;

	const int baseRadius=10;
	for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++) {
		const AngleRadians angle = static_cast<double>(i) / CIRCLE_RESOLUTION
				* TAU;
		base_circle.emplace_back(cos(angle) * baseRadius, sin(angle) * baseRadius);
	}

	const coord_t z_distance_top = mesh.settings.get<coord_t>("support_top_distance");
	const size_t z_distance_top_layers =( round_up_divide(z_distance_top, config.layer_height) + 1); //Support must always be 1 layer below overhang.
	const coord_t support_roof_line_width=mesh.settings.get<coord_t>("support_roof_line_width");
	const coord_t support_line_width=mesh.settings.get<coord_t>("support_line_width");

	const coord_t support_roof_line_distance=mesh.settings.get<coord_t>("support_roof_line_distance");
	const coord_t minimum_roof_area=mesh.settings.get<coord_t>("minimum_roof_area")*1000;
	const coord_t support_tree_branch_distance =mesh.settings.get<coord_t>("support_tree_branch_distance");
	const bool roof_enabled=config.dont_move_until_ddt!=0;//todo
	const bool only_gracious=SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL;
	std::vector<AngleDegrees> support_infill_angles;
	std::vector<AngleDegrees> support_roof_angles;
	support_infill_angles=mesh.settings.get<std::vector<AngleDegrees>>("support_infill_angles");
	support_roof_angles=mesh.settings.get<std::vector<AngleDegrees>>("support_roof_angles");


	const coord_t connect_length = 2*(config.getRadius(1,0)+config.maximum_move_distance_slow); // i assume one only wants lines of distance support_tree_branch_distance. If both dimensions should be support_tree_branch_distance change connect length to support_tree_branch_distance
	const coord_t outset=roof_enabled?mesh.settings.get<coord_t>("support_roof_offset"):mesh.settings.get<coord_t>("support_offset");

	if(support_infill_angles.size() == 0){
		support_infill_angles.push_back(45);
		support_infill_angles.push_back(135);
	}

	if(support_roof_angles.size() == 0){
		support_roof_angles.push_back(45);
		support_roof_angles.push_back(135);
	}

	logDebug("Distance between points of lines of inital influene areas %lld \n",connect_length);
	std::vector<std::unordered_set<Point>> already_inserted(mesh.overhang_areas.size() - z_distance_top_layers);

	if(mesh.overhang_areas.size()<=z_distance_top_layers)
		return;

#pragma omp parallel for
	for (coord_t layer_nr =  mesh.overhang_areas.size() - z_distance_top_layers -1 ; layer_nr >=1;layer_nr--) {
		if(mesh.overhang_areas[layer_nr+ z_distance_top_layers].empty())
			continue;

		const Polygons outline =storage.getLayerOutlines(layer_nr+ z_distance_top_layers, false, false, false);

		Polygons relevant_forbidden=(config.support_rests_on_model?(only_gracious?volumes_.getAvoidance(config.getRadius(1),layer_nr,false,true):volumes_.getCollision(config.getRadius(1), layer_nr)):volumes_.getAvoidance(config.getRadius(1),layer_nr));

		Polygons overhang = mesh.overhang_areas[layer_nr+ z_distance_top_layers].difference(relevant_forbidden);

		if(overhang.empty())
			continue;

		//Polygons overhang_outset=overhang.offset(outset).difference(relevant_forbidden);

		for(Polygons overhang_outset:overhang.offset(outset).difference(relevant_forbidden).splitIntoParts(true) ){

		std::function<Polygons(const Polygons&,bool,size_t)> generateLines=[&](const Polygons& area,bool roof,size_t dtt){
			Polygons gaps;
			//as we effectivly use lines to place our supportPoints we may use the Infill class for it, while not made for it it works perfect
			const EFillMethod pattern=roof?mesh.settings.get<EFillMethod>("support_roof_pattern"):mesh.settings.get<EFillMethod>("support_pattern");

			const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
			const bool connect_polygons = false;
			constexpr coord_t support_roof_overlap = 0; // the roofs should never be expanded outwards
			constexpr size_t infill_multiplier = 1;
			constexpr coord_t outline_offset =  0;
			constexpr coord_t extra_infill_shift = 0;
			constexpr size_t wall_line_count = 0;
			const Point infill_origin;
			constexpr Polygons* perimeter_gaps = nullptr;
			constexpr bool use_endpieces = true;
			constexpr bool connected_zigzags = false;
			constexpr bool skip_some_zags = false;
			constexpr size_t zag_skip_count = 0;
			constexpr coord_t pocket_size = 0;
			std::vector<AngleDegrees> angles = roof ? support_roof_angles:support_infill_angles;

			const AngleDegrees fill_angle=angles[(layer_nr-dtt)%angles.size()];
			const coord_t z=layer_nr-dtt;
		    Infill roof_computation(
		        pattern, zig_zaggify_infill, connect_polygons, area, outline_offset, roof?support_roof_line_width:support_line_width,
		        roof?support_roof_line_distance:support_tree_branch_distance, support_roof_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
		        wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
		        );
		    Polygons areas;
			Polygons lines;
			roof_computation.generate(areas, lines, storage.support.cross_fill_provider,&mesh);
			lines.add(toPolylines(areas));
			return lines;
		};


		std::function<void(std::pair<Point,LineStatus>,size_t,coord_t)> addToData=[&](std::pair<Point,LineStatus> p,size_t ddt, coord_t insert_layer) {

			bool to_bp=p.second==LineStatus::TO_BP;
			bool gracious = to_bp|| p.second==LineStatus::TO_MODEL_GRACIOUS;
			if(!config.support_rests_on_model&&!to_bp){
				return;
			}

			Polygon circle;
			Polygon outer_circle;
			for (Point corner : base_circle){
				circle.add(p.first + corner);
				outer_circle.add(p.first +corner*(10+((volumes_.ceilRadius(config.getRadius(ddt))*1.0)/baseRadius))); // adds 100 microns more to the radius as otherwise the draw areas may fail to ovalize
			}
#pragma omp critical(moveBounds)
			{
				if(!already_inserted[insert_layer].count(p.first/((support_roof_line_width+1)/10))){//we normalize the point a bit to also catch points which are so close that inserting it would achieve nothing todo orrect lw
					already_inserted[insert_layer].emplace(p.first/((support_roof_line_width+1)/10));
					SupportElement* elem= new SupportElement(ddt,insert_layer,p.first,to_bp,gracious); // +1 as we otherwise would
					elem->area=new Polygons(circle.offset(0));
					Polygons* outer_wall_area=new Polygons(outer_circle.offset(0)); // technically wrong by up to base_radius, but i dont care about 10 microns
					move_bounds[insert_layer].emplace(elem,outer_wall_area);
				}
			}
		};

		std::vector<std::vector<LineInformation>> cache(support_roof_angles.size());
		std::function<void(size_t,bool)> prepareData=[&](size_t dtt,bool roof){
			if(dtt!=0){
				Polygons forbidden =(config.support_rests_on_model?(only_gracious?volumes_.getAvoidance(config.getRadius(1),layer_nr-dtt,false,true):volumes_.getCollision(config.getRadius(1), layer_nr-dtt)):volumes_.getAvoidance(config.getRadius(1),layer_nr-dtt));
				overhang_outset=overhang_outset.difference(forbidden);

			}
			if(cache[(layer_nr-dtt)%cache.size()].empty()){
				Polygons lines = generateLines(overhang_outset,roof,dtt);
				std::vector<LineInformation> internal_lines = convertLinesToInternal(ensureMaximumDistancePolyline(lines,connect_length),layer_nr-dtt);
				cache[(layer_nr-dtt)%cache.size()]=internal_lines;
			}
			else{
				cache[(layer_nr-dtt)%cache.size()]=splitLines(cache[(layer_nr-dtt)%cache.size()], layer_nr+1-dtt, dtt).first;
			}
		};

		bool continue_outer=false;

		for(size_t i=0;i<config.dont_move_until_ddt&&layer_nr-i>=1;i++){
			prepareData(i,true);
			std::pair<std::vector<LineInformation>,std::vector<LineInformation>> splits=splitLines(cache[(layer_nr-i)%cache.size()], layer_nr-i, i+1);
			for(auto line :splits.second){

				for(auto p:line){
					addToData(p,1,layer_nr-i);
				}
			}
			cache[(layer_nr-i)%cache.size()]=splits.first;
			if(overhang_outset.area()>minimum_roof_area){ // todo ||cache[(layer_nr-i)%cache.size()].empty?

				#pragma omp critical(precalculatedSupportLayers)
				{
					storage.support.supportLayers[layer_nr-i].support_roof.add(overhang_outset);
				}
			}
			else{
				size_t points_added=0;
				for(LineInformation line:splits.first){
					for(auto point_data:line)
					{
						points_added++;
						addToData(point_data,1,layer_nr-i);
					}
				}

				if(points_added<3&&i==0){// as we did not add enough points we also add the outline as points to ensure small areas are properly supported
					for(LineInformation line:convertLinesToInternal(ensureMaximumDistancePolyline(toPolylines(overhang_outset.offset(-5)),connect_length), layer_nr)){
						for(auto point_data:line)
						{
							addToData(point_data,1,layer_nr-i);
						}
					}

				}
				continue_outer=true;
				break;
			}
		}
		if(continue_outer){
			continue;
		}

		if(static_cast<coord_t>(config.dont_move_until_ddt)<layer_nr){

			prepareData(config.dont_move_until_ddt,roof_enabled);
			std::vector<LineInformation> last_lines = cache[(layer_nr-config.dont_move_until_ddt)%cache.size()];
			for(LineInformation line:last_lines){
				for(auto point_data:line){
					addToData(point_data,1,layer_nr-config.dont_move_until_ddt);
				}
			}
		}

		else{
#pragma omp critical(precalculatedSupportLayers)
{
			storage.support.supportLayers[0].support_roof.add(overhang_outset);
}
		}
	}
	}

}

ModelVolumes::ModelVolumes(const SliceDataStorage& storage, coord_t xy_distance, coord_t max_move,coord_t max_move_slow ,coord_t radius_sample_resolution,coord_t z_distance_bottom_layers,
		coord_t z_distance_top_layers,bool support_rests_on_model,bool avoid_support_blocker,bool use_exponential_collision_resolution,coord_t exponential_threashold,double exponential_factor) :
		machine_border_ { calculateMachineBorderCollision(
				storage.getMachineBorder()) }, xy_distance_ { xy_distance }, max_move_ {max_move },max_move_slow {max_move_slow}, radius_sample_resolution_ { radius_sample_resolution },z_distance_bottom_layers{z_distance_bottom_layers},
				z_distance_top_layers{z_distance_top_layers},support_rests_on_model{support_rests_on_model},avoid_support_blocker{avoid_support_blocker},use_exponential_collision_resolution{use_exponential_collision_resolution},
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
	}
	else {
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


void ModelVolumes::precalculateAvoidance(const size_t max_layer,const coord_t branch_radius, const double scale_factor,const coord_t min_max_radius,const double scale_foot) {

	size_t min_max_height =0;
	double progress_total =0;
	for(coord_t counter=0;branch_radius+branch_radius*counter*scale_foot<min_max_radius;counter++){
		min_max_height=counter;
	}

	if(layer_outlines_.size()==0)
		return;

	const size_t model_height= layer_outlines_.size()-1;
	std::function<size_t(coord_t,coord_t)> getRequiredHeight=[&](coord_t radius,coord_t radius_before) {
		size_t max_required_layer=max_layer;
		if(radius>branch_radius){
			if(scale_factor>0){

				double layers_handled_by_rad_before=(radius_before-branch_radius)/(branch_radius*scale_factor);
				max_required_layer-=layers_handled_by_rad_before-2; // -2 to avoid off by one problems and i dont care about a few layers more
			}
		}
		if(max_required_layer<min_max_height) max_required_layer=min_max_height;
		if(max_required_layer>model_height) max_required_layer=model_height;
		if(max_required_layer>max_layer) max_required_layer=max_layer;

		return max_required_layer;
	};
	std::vector<int> collision_radii; // todo remove add to resolution steps
	collision_radii.push_back(0);

	const coord_t max_radius=std::max(ceilRadius( branch_radius * (max_layer+1) * scale_factor+branch_radius),ceilRadius(min_max_radius));
	for (coord_t radius=radius_sample_resolution_;radius<= max_radius;radius=ceilRadius(radius+1)){
		resolution_steps.push_back(radius);
		collision_radii.push_back(radius);
	}

	std::vector<Polygons> prePlaceable(max_layer+1);
	//as we dont have multiple radiis for the support blocker we can not parallelize it
	if(avoid_support_blocker)
	for(size_t i=0;i<getRequiredHeight(0,0)-1&&i<anti_overhang_.size();i++){
		if(!anti_overhang_[i].empty()){
			anti_overhang_[i].simplify(30, 10);
			anti_overhang_[i+1]=anti_overhang_[i+1].unionPolygons(anti_overhang_[i].offset(-max_move_, ClipperLib::jtRound)); // we create a anti_overhang avoidance => because of this we can union it with the collision later
		}

	}

	// calculate all required collisions
#pragma omp parallel for schedule(static,1) shared(collision_cache_,prePlaceable)
	for(long long unsigned int i=0;i<collision_radii.size();i++){
		coord_t radius = collision_radii[i];

		size_t max_required_layer=getRequiredHeight(radius,i!=0?collision_radii[i-1]:0);
		if(radius==0) max_required_layer=model_height; // we need r=0 everywhere
		std::vector<std::pair<RadiusLayerPair,Polygons>> data(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons())); // we need to set a invalid default element because sometines we seem to add one


		logDebug("precalculating collision: radius: %lld up to layer %lld\n",radius,max_required_layer);
		RadiusLayerPair key(radius,0);

		for(size_t layer=0;layer<=max_required_layer;layer++){
			key.second=layer;
			Polygons collision_areas = machine_border_;
			if (layer < layer_outlines_.size()) {
				collision_areas = collision_areas.unionPolygons(layer_outlines_[layer]);
			}
			collision_areas = collision_areas.offset(xy_distance_ + radius, ClipperLib::JoinType::jtRound);

			collision_areas.simplify(30,10);

			data[layer]=std::pair<RadiusLayerPair,Polygons>(key,collision_areas);

		}


		for(coord_t layer=static_cast<coord_t>(max_required_layer);layer>=0;layer--){ // have to do signed as we are counting down
			for(coord_t i=1;i<=z_distance_bottom_layers&&layer-i>0;i++){
				data[layer].second=data[layer].second.unionPolygons(data[layer-i].second);
			}
			if(support_rests_on_model&& layer+1<static_cast<coord_t>(prePlaceable.size())&&layer<static_cast<coord_t>(max_required_layer)&&radius==0){
				prePlaceable[layer+1]=data[layer].second.difference(data[layer+1].second);
			}
		}

		for(size_t layer=0;layer<=max_required_layer;layer++){

			for(coord_t i=1;i<= z_distance_top_layers && i+layer <=max_required_layer;i++){
				data[layer].second=data[layer].second.unionPolygons(data[layer+i].second);
			}
			//we add the anti overhang to the collision, as this anti_overhang was already converted into an anti_overhang avoidance we can just add it as we can always escape this avoidance with max move speed
			if(avoid_support_blocker&&!anti_overhang_[layer].empty())
				data[layer].second=data[layer].second.unionPolygons(anti_overhang_[layer].offset(radius, ClipperLib::JoinType::jtRound));// we add the support blocker to ensure there is no support inside the support blocker
		}

		#pragma omp critical(collision_cache_)
		{
			collision_cache_.insert(data.begin(),data.end());
			const double progress_step = PROGRESS_PRECALC_COLL/collision_radii.size();
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
		size_t max_required_layer=getRequiredHeight(radius,i!=0?resolution_steps[i-1]:0);

		std::vector<std::pair<RadiusLayerPair,Polygons>> data(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
		std::vector<std::pair<RadiusLayerPair,Polygons>> data_slow(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));

		Polygons last_layer=getCollision(radius,0);
		Polygons last_layer_slow=getCollision(radius,0);
		last_layer.simplify(30,10);
		last_layer_slow.simplify(30,10);
		data[0]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);
		data_slow[0]=std::pair<RadiusLayerPair,Polygons>(key,last_layer_slow);

		logDebug("precalculating regular avoidance: radius: %lld up to layer %lld\n",radius,max_required_layer);

		for(size_t layer=1;layer<=max_required_layer;layer++){
			key.second=layer;
			Polygons col=getCollision(radius,layer);

			last_layer=safeOffset(last_layer,-max_move_,ClipperLib::jtRound,-max_step_move,col).smooth(5);
			last_layer.simplify(30,10);
			data[layer]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);

			last_layer_slow=safeOffset(last_layer_slow,-max_move_slow,ClipperLib::jtRound,-max_step_move,col).smooth(5);
			last_layer_slow.simplify(30,10);
			data_slow[layer]=std::pair<RadiusLayerPair,Polygons>(key,last_layer_slow);

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
	if(support_rests_on_model){

	#pragma omp for schedule(static,1) // to ensure all cores get equal amount of stuff to calculate schedule guided ? dynamic mit hoher blocksize TODO
		for(long long unsigned int i=0;i<resolution_steps.size();i++){

			coord_t radius = resolution_steps[i];

			const coord_t max_step_move = 2*(radius+ xy_distance_);

			RadiusLayerPair key(radius,0);
			size_t max_required_layer=getRequiredHeight(radius,i!=0?resolution_steps[i-1]:0);

			std::vector<std::pair<RadiusLayerPair,Polygons>> data(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			std::vector<std::pair<RadiusLayerPair,Polygons>> data_slow(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			std::vector<std::pair<RadiusLayerPair,Polygons>> data_placeable(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			Polygons last_layer=getCollision(radius,0);
			Polygons last_layer_slow=getCollision(radius,0);
			last_layer.simplify(30,10);
			last_layer_slow.simplify(30,10);
			data[0]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);
			data_slow[0]=std::pair<RadiusLayerPair,Polygons>(key,last_layer_slow);

			logDebug("precalculating avoidance to model: radius: %lld up to layer %lld\n",radius,max_required_layer);

			for(size_t layer=1;layer<=max_required_layer;layer++){
				//printf("precalc: avoidance layer %d vertexes: %d\n",layer,last_layer.pointCount());
				key.second=layer;
				Polygons placeable=prePlaceable[layer].offset(-radius,ClipperLib::jtRound);

				Polygons col=getCollision(radius,layer);

				last_layer=safeOffset(last_layer,-max_move_,ClipperLib::jtRound,-max_step_move,col).difference(placeable).smooth(5);
				last_layer_slow=safeOffset(last_layer_slow,-max_move_slow,ClipperLib::jtRound,-max_step_move,col).difference(placeable).smooth(5);// removed .unionPolygons(getCollision(radius, layer))

				last_layer.simplify(30,10);
				last_layer_slow.simplify(30,10);
				data[layer]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);
				data_slow[layer]=std::pair<RadiusLayerPair,Polygons>(key,last_layer_slow);
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

Polygons ModelVolumes::calculateMachineBorderCollision(Polygon machine_border) {
	Polygons machine_volume_border;
	machine_volume_border.add(machine_border.offset(1000000)); //Put a border of 1m around the print volume so that we don't collide.
	machine_border.reverse(); //Makes the polygon negative so that we subtract the actual volume from the collision area.
	machine_volume_border.add(machine_border);
	return machine_volume_border;
}

} //namespace cura
