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

#include <optional>

#include <string>
#include <numeric>
#include <fstream>
#include "support.h"

#define CIRCLE_RESOLUTION 25 //The number of vertices in each circle.

//The various stages of the process can be weighted differently in the progress bar.
//These weights are obtained experimentally.
#define PROGRESS_PRECALC_COLL 1000
#define PROGRESS_PRECALC_AVO 4000
#define PROGRESS_GENERATE_NODES 1000
#define PROGRESS_AREA_CALC 3000
#define PROGRESS_DRAW_AREAS 1000

#define PROGRESS_TOTAL 10000

#define SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL false
#define SUPPORT_TREE_AVOID_SUPPORT_BLOCKER true
#define SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION true
#define SUPPORT_TREE_EXPONENTIAL_THREASHOLD 1000
#define SUPPORT_TREE_EXPONENTIAL_FACTOR 1.25
#define SUPPORT_TREE_COLLISION_RESOLUTION 250
#define SUPPORT_TREE_ADD_REGULAR_POINTS_ADDITIONALLY_ON_SMALL_AREAS true
#define SUPPORT_TREE_MAX_DEVIATION 0

namespace cura {

TreeSupport::TreeSupport(const SliceDataStorage& storage) {
	for(size_t mesh_idx=0;mesh_idx<storage.meshes.size();mesh_idx++){ //group all meshes that can be processed together
		SliceMeshStorage mesh = storage.meshes[mesh_idx];

		if (!storage.meshes[mesh_idx].settings.get<bool>("support_tree_enable")) {
			continue;
		}

		bool added=false;
		TreeSupportSettings next_settings(mesh.settings);
		for(size_t idx =0;idx<grouped_meshes.size();idx++){
			if(next_settings.hasSameInfluenceAreaSettings(grouped_meshes[idx].first)){
				added=true;
				grouped_meshes[idx].second.emplace_back(mesh_idx);
				//handle some settings that are only used for performance reasons. This ensures that a horrible set setting intended to improve performance can not reduce it drastically.
				grouped_meshes[idx].first.performance_interface_skip_layers=std::min(grouped_meshes[idx].first.performance_interface_skip_layers,next_settings.performance_interface_skip_layers);
			}
		}
		if(!added){
			grouped_meshes.emplace_back(next_settings,std::vector<size_t>{mesh_idx});
		}

	}
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

	size_t counter=0;
	for(std::pair<TreeSupportSettings,std::vector<size_t>>processing:grouped_meshes){ //process each combination of meshes

		std::vector<std::map<SupportElement*,Polygons*>> move_bounds(storage.support.supportLayers.size()); // value is the area where support may be placed. As this is calculated in CreateLayerPathing it is saved and reused in drawAreas
		log("Processing support tree mesh group %lld of %lld containing %lld meshes.\n",counter+1,grouped_meshes.size(),grouped_meshes[counter].second.size());
		std::vector<Polygons> exclude(storage.support.supportLayers.size());
	    auto t_start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
		for(LayerIndex layer_idx=0;layer_idx<coord_t(storage.support.supportLayers.size());layer_idx++){ //get all already existing support areas and exclude them
			Polygons exlude_at_layer;
			exlude_at_layer.add(storage.support.supportLayers[layer_idx].support_bottom);
			exlude_at_layer.add(storage.support.supportLayers[layer_idx].support_roof);
			for(auto part:storage.support.supportLayers[layer_idx].support_infill_parts){
				exlude_at_layer.add(part.outline);
			}
			exclude[layer_idx]=exlude_at_layer.unionPolygons();
		}
		config=processing.first; //this struct is used to easy retrieve setting. No other function except those in ModelVolumes and generateInitalAreas have knowledge of the existence of multiple meshes being processed.

	    progress_multiplier=1.0/double(grouped_meshes.size());
	    progress_offset=counter==0?0:PROGRESS_TOTAL*(double(counter)*progress_multiplier);
	    volumes_=ModelVolumes(storage, config.maximum_move_distance, config.maximum_move_distance_slow, processing.second.front(),progress_multiplier,progress_offset,exclude);
	    precalculate(storage,processing.second); //generate avoidance areas
	    auto t_precalc = std::chrono::high_resolution_clock::now();

		for(size_t mesh_idx:processing.second){
			generateInitalAreas(storage.meshes[mesh_idx],move_bounds,storage);
		}

		auto t_gen = std::chrono::high_resolution_clock::now();
		//Propagate the influence areas downwards.
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
		auto dur_total = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_draw - t_start ).count();
		log("Total time used creating Tree support for the currently grouped meshes: %.3lf ms. Different subtasks:\nCalculating Avoidance: %.3lf ms Creating inital influence areas: %.3lf ms Influence area creation: %.3lf ms Placement of Points in InfluenceAreas: %.3lf ms Drawing result as support %.3lf ms\n",dur_total,dur_pre_gen,dur_gen,dur_path,dur_place,dur_draw);

		//for(int i=10;i<50;i++)
		//	outputLayerAsSvg(move_bounds,i,storage);

		for (auto& layer : move_bounds) {
			for (auto elem : layer){
				delete elem.second;
				delete elem.first->area;
				delete elem.first;
			}
		}

		counter++;
	}

    storage.support.generated = true;
}



coord_t TreeSupport::precalculate(SliceDataStorage& storage,std::vector<size_t> currently_processing_meshes){

	size_t max_layer=0;
	for (size_t mesh_idx : currently_processing_meshes) {
		SliceMeshStorage& mesh = storage.meshes[mesh_idx];
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

	volumes_.precalculate(max_layer);
	return max_layer;
}



std::vector<TreeSupport::LineInformation> TreeSupport::convertLinesToInternal(Polygons polylines,coord_t layer_nr){
	std::vector<LineInformation> result;
	const bool xy_overrides=config.support_overrides == SupportDistPriority::XY_OVERRIDES_Z;

	// we check if the position is valid, if it is NOT we delete that point
	for(auto line:polylines){
		LineInformation res_line;
		for(Point p:line){
			if(!volumes_.getAvoidance(config.getRadius(0), layer_nr,false,false,!xy_overrides).inside(p, true)){
				res_line.emplace_back(p,LineStatus::TO_BP);
			}
			else if(config.support_rests_on_model&&!volumes_.getAvoidance(config.getRadius(0), layer_nr,false,true,!xy_overrides).inside(p, true)){
				res_line.emplace_back(p,LineStatus::TO_MODEL_GRACIOUS);
			}
			else if(config.support_rests_on_model&&!volumes_.getCollision(config.getRadius(0), layer_nr,!xy_overrides).inside(p, true)){
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

	const bool xy_overrides=config.support_overrides == SupportDistPriority::XY_OVERRIDES_Z;

	std::vector<LineInformation> keep(1);
	std::vector<LineInformation> set_free(1);
	enum STATE{keeping,freeing};

	std::function<bool(std::pair<Point,LineStatus>)> evaluatePoint=[&](std::pair<Point,LineStatus> p) {
		if(!volumes_.getAvoidance(config.getRadius(ddt+1), current_layer-1,false,false,!xy_overrides).inside(p.first, true)){
			return true;
		}
		if(config.support_rests_on_model&&p.second!=LineStatus::TO_BP){

			if( p.second==LineStatus::TO_MODEL_GRACIOUS){
				return !volumes_.getAvoidance(config.getRadius(ddt+1), current_layer-1,false,true,!xy_overrides).inside(p.first, true);
			}
			else{
				return !volumes_.getCollision(config.getRadius(ddt+1), current_layer-1,!xy_overrides).inside(p.first, true);
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


void writePolylines(SVG& svg,Polygons polylines,SVG::Color color){ //todo remove as only for debugging relevant
	for(auto path :polylines){
		if(path.size()==0)
		{
			continue;
		}
		Point before = path[0];
		for(size_t i=1;i<path.size();i++){
			svg.writeLine(before, path[i], color, 2);
			before=path[i];
		}
	}
}


//ensures that every line segment is about distance in length. The resulting lines may differ from the original but all points are on the original
Polygons TreeSupport::ensureMaximumDistancePolyline (const Polygons& input, coord_t distance,size_t min_points)const{
	Polygons result;
	for(auto part:input){
		if(part.size()==0)
		{
			continue;
		}
		coord_t length = Polygon(part).offset(0).polyLineLength();
		size_t points=length/distance;
		if(points < min_points){
			points =min_points;
		}
		coord_t target_distance=length/points;
		Polygon line;
		if(points==1){
			target_distance=length/2+10; // we will only have the point in the middle
		}
		else{
			line.add(part[0]);
		}
		coord_t used_distance=0;
		for(size_t pos=0;pos+1<part.size();pos++){
			Point me =part[(part.size()+pos)%part.size()];
			Point next=part[(part.size()+pos+1)%part.size()];
			coord_t next_step_length;
			do{
				next_step_length=vSize(me-next);
				if(next_step_length+used_distance>=target_distance){
					double scale = (1.0*(target_distance-used_distance))/(next_step_length);
					me= me + (next-me)*scale;
					used_distance=0;
					line.add(me);
					continue;
				}
				used_distance+=next_step_length;
			}
			while(next_step_length>target_distance);
		}
		if(points>1){
			line.add(part.back());
		}
		result.add(line);

	}
	return result;
}

//adds the implizit line from the last vertex to the first explicitly
Polygons TreeSupport::toPolylines(const Polygons& poly)const{
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

	TreeSupportSettings mesh_config(mesh.settings);

	const size_t z_distance_top_layers =mesh_config.z_distance_top_layers+1;
	const coord_t support_roof_line_width=mesh.settings.get<coord_t>("support_roof_line_width");
	const coord_t support_line_width=mesh.settings.get<coord_t>("support_line_width");

	const coord_t support_roof_line_distance=mesh.settings.get<coord_t>("support_roof_line_distance");
	const coord_t minimum_roof_area=mesh.settings.get<coord_t>("minimum_roof_area")*1000;
	const coord_t support_tree_branch_distance = support_line_width * 100/mesh.settings.get<double>("support_tree_top_rate");
	const bool roof_enabled=mesh_config.support_roof_layers!=0;
	const bool only_gracious=SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL;
	const EFillMethod roof_pattern =mesh.settings.get<EFillMethod>("support_roof_pattern");
	const EFillMethod support_pattern =mesh.settings.get<EFillMethod>("support_pattern");
	const coord_t connect_length = support_tree_branch_distance;
	const coord_t outset=(mesh_config.performance_increased_xy_min?100:0) + (roof_enabled?mesh.settings.get<coord_t>("support_roof_offset"):mesh.settings.get<coord_t>("support_offset"));
	const double support_overhang_angle=mesh.settings.get<AngleRadians>("support_angle");
	const coord_t max_overhang_speed=(support_overhang_angle < TAU / 4) ? (coord_t) (tan(support_overhang_angle) * mesh_config.layer_height) : std::numeric_limits<coord_t>::max();
	const size_t max_overhang_insert_lag=std::max((size_t)round_up_divide(mesh_config.xy_distance,max_overhang_speed),mesh_config.z_distance_top_layers); //cap for how much layer below the target a new support point may be added, as other than with regular support every new inserted point may cause extra material and time cost.  Could also be an user setting or differently calculated.
	const bool xy_overrides=mesh_config.support_overrides == SupportDistPriority::XY_OVERRIDES_Z;
	std::vector<AngleDegrees> support_infill_angles;
	std::vector<AngleDegrees> support_roof_angles;
	support_infill_angles=mesh.settings.get<std::vector<AngleDegrees>>("support_infill_angles");
	support_roof_angles=mesh.settings.get<std::vector<AngleDegrees>>("support_roof_angles");

	std::function<void(std::vector<AngleDegrees>&,EFillMethod)> getInterfaceAngles=[&](std::vector<AngleDegrees>& angles,EFillMethod pattern){ // (logic) from getInterfaceAngles in FFFGcodeWriter.
		if (angles.empty())
		{
			if (pattern == EFillMethod::CONCENTRIC)
			{
				angles.push_back(0); //Concentric has no rotation.
			}
			else if (pattern == EFillMethod::TRIANGLES)
			{
				angles.push_back(90); //Triangular support interface shouldn't alternate every layer.
			}
			else
			{
				if (mesh_config.support_roof_layers>=2)
				{
					//Some roofs are quite thick.
					//Alternate between the two kinds of diagonal: / and \ .
					angles.push_back(45);
					angles.push_back(135);
				}
				if (angles.empty())
				{
					angles.push_back(90); //Perpendicular to support lines.
				}
			}
		}
	};

	getInterfaceAngles(support_infill_angles,support_pattern);
	getInterfaceAngles(support_roof_angles,roof_pattern);

	if(mesh.overhang_areas.size() <= z_distance_top_layers){
		return;
	}
	logDebug("Distance between points of lines of inital influence areas %lld \n",connect_length);
	std::vector<std::unordered_set<Point>> already_inserted(mesh.overhang_areas.size() - z_distance_top_layers);
	if(mesh.overhang_areas.size()<=z_distance_top_layers){
		return;
	}


#pragma omp parallel for schedule(dynamic)
	for (coord_t layer_nr_outer =  mesh.overhang_areas.size() - z_distance_top_layers -1 ; layer_nr_outer >=1;layer_nr_outer--) { // counting down as it is more likely to encounter areas that need much support at the top then at the bottom.
		if(mesh.overhang_areas[layer_nr_outer+ z_distance_top_layers].empty())
		{
			continue;
		}
		Polygons relevant_forbidden=(mesh_config.support_rests_on_model?(only_gracious?volumes_.getAvoidance(mesh_config.getRadius(0),layer_nr_outer,false,true,!xy_overrides):volumes_.getCollision(mesh_config.getRadius(0), layer_nr_outer,!xy_overrides)):volumes_.getAvoidance(mesh_config.getRadius(0),layer_nr_outer,false,false,!xy_overrides));

		std::vector<std::pair<LayerIndex,Polygons>> overhang_processing;
		if(xy_overrides){
		Polygons removed_overhang=mesh.overhang_areas[layer_nr_outer+ z_distance_top_layers].intersection(relevant_forbidden);
			for(size_t lag_ctr=1;lag_ctr<=max_overhang_insert_lag&&removed_overhang.area()>1&&layer_nr_outer-coord_t(lag_ctr)>=1;lag_ctr++){
				Polygons relevant_forbidden_below=(mesh_config.support_rests_on_model?(only_gracious?volumes_.getAvoidance(mesh_config.getRadius(0),layer_nr_outer-lag_ctr,false,true,!xy_overrides):volumes_.getCollision(mesh_config.getRadius(0), layer_nr_outer-lag_ctr,!xy_overrides)):volumes_.getAvoidance(mesh_config.getRadius(0),layer_nr_outer-lag_ctr,false,false,!xy_overrides));
				Polygons regained_overhang=removed_overhang.difference(relevant_forbidden_below);
				if(regained_overhang.area()>1){
					for(Polygons regained_overhang_part:regained_overhang.offset(outset>0?outset:0).splitIntoParts(false) ){
						overhang_processing.emplace_back(layer_nr_outer-lag_ctr,regained_overhang_part);
					}
					removed_overhang=removed_overhang.intersection(relevant_forbidden_below);
				}
				else{ // no overhang area was gained back as such it can be assumed that no area overhangs anymore.
					break;
				}
			}
		}


		Polygons overhang = mesh.overhang_areas[layer_nr_outer+ z_distance_top_layers].difference(relevant_forbidden);
		for(Polygons overhang_outset:overhang.offset(outset>0?outset:0).difference(relevant_forbidden).splitIntoParts(true) ){
			overhang_processing.emplace_back(layer_nr_outer,overhang_outset);
		}
		if(overhang_processing.empty())
		{
			continue;
		}

		if(outset<0){
			std::vector<std::pair<LayerIndex,Polygons>> revised_processing;
			for(std::pair<LayerIndex,Polygons> processing_pair:overhang_processing){
				Polygons overhang_outset_smaller=processing_pair.second.offset(outset);
					if(overhang_outset_smaller.area()<=1){ // if the outset is applied the area would disappear so the original is kept.
						revised_processing.emplace_back(processing_pair);
					}
					else{
						for(Polygons overhang_part:overhang_outset_smaller.splitIntoParts(false) ){ //another check for parts is required,a ss the negative offset may have created more parts
							revised_processing.emplace_back(processing_pair.first,overhang_part);
						}
					}
			}
			overhang_processing=revised_processing;
		}

		for(std::pair<LayerIndex,Polygons> processing_pair:overhang_processing){
			Polygons overhang_outset= processing_pair.second;
			LayerIndex layer_idx=processing_pair.first;

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
			const coord_t support_infill_distance=roof?support_roof_line_distance:support_tree_branch_distance*(pattern==EFillMethod::TRIANGLES?3:(pattern==EFillMethod::GRID?2:1));
			std::vector<AngleDegrees> angles = roof ? support_roof_angles:support_infill_angles;

			const coord_t z=layer_idx-dtt;
	        int divisor = static_cast<int>(angles.size());
	        int index = ((z % divisor) + divisor) % divisor;
			const AngleDegrees fill_angle=angles[index];
		    Infill roof_computation(
		        pattern, zig_zaggify_infill, connect_polygons, area, outline_offset, roof?support_roof_line_width:support_line_width,
		        support_infill_distance, support_roof_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
		        wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
		        );
		    Polygons areas;
			Polygons lines;
			roof_computation.generate(areas, lines);
			lines.add(toPolylines(areas));
			return lines;
		};

		std::function<void(std::pair<Point,LineStatus>,size_t,coord_t,size_t,bool)> addToData=[&](std::pair<Point,LineStatus> p,size_t dtt, coord_t insert_layer,size_t dont_move_until,bool roof) {
			bool to_bp=p.second==LineStatus::TO_BP;
			bool gracious = to_bp|| p.second==LineStatus::TO_MODEL_GRACIOUS;
			if(!mesh_config.support_rests_on_model&&!to_bp){
				logWarning("Tried to add an invalid support point\n");
				return;
			}

			Polygon circle;
			for (Point corner : base_circle){
				circle.add(p.first + corner);
			}
#pragma omp critical(moveBounds)
			{
				if(!already_inserted[insert_layer].count(p.first/((support_line_width+1)/10))){//we normalize the point a bit to also catch points which are so close that inserting it would achieve nothing
					already_inserted[insert_layer].emplace(p.first/((support_line_width+1)/10));
					SupportElement* elem= new SupportElement(dtt,insert_layer,p.first,to_bp,gracious,!xy_overrides,dont_move_until,roof);
					elem->area=new Polygons(circle.offset(0));
					Polygons* outer_wall_area=new Polygons(circle.offset(0)); //as it would be required to subtract the collision, this is done in the draw areas instead, as there is is better parallelized
					move_bounds[insert_layer].emplace(elem,outer_wall_area);
				}
			}
		};

		std::vector<LineInformation> overhang_lines;

		Polygons last_overhang=overhang_outset;
		size_t dtt_roof=0;
		for(dtt_roof=0;dtt_roof<mesh_config.support_roof_layers&&layer_idx-dtt_roof>=1;dtt_roof++){// here the roof is handled. If roof can not be added the branches will try to not move instead
			Polygons forbidden_next =(mesh_config.support_rests_on_model?(only_gracious?volumes_.getAvoidance(mesh_config.getRadius(0),layer_idx-(dtt_roof+1),false,true,!xy_overrides):volumes_.getCollision(mesh_config.getRadius(0), layer_idx-(dtt_roof+1),!xy_overrides)):volumes_.getAvoidance(mesh_config.getRadius(0),layer_idx-(dtt_roof+1),false,false,!xy_overrides));
			Polygons overhang_outset_next=overhang_outset.difference(forbidden_next);
			if(overhang_outset_next.area()<minimum_roof_area){ // next layer down the roof area would be to small so we have to insert our roof support here

				size_t dtt_before=dtt_roof>0?dtt_roof-1:0;
				if(SUPPORT_TREE_ADD_REGULAR_POINTS_ADDITIONALLY_ON_SMALL_AREAS||dtt_roof!=0){
					overhang_lines=convertLinesToInternal(ensureMaximumDistancePolyline(generateLines(last_overhang,roof_enabled,dtt_before),connect_length,1),layer_idx-dtt_before);
				}
				if(dtt_roof!=0){
					overhang_lines=splitLines(overhang_lines, layer_idx-dtt_before, dtt_before).first;
				}
				else{// if the area is so small at dtt_roof 0 that a roof is impossible extra support points are added to ensure the overhang is supported.
					std::vector<LineInformation> extra_lines=convertLinesToInternal(ensureMaximumDistancePolyline(toPolylines(overhang_outset),connect_length,3),layer_idx-dtt_roof);
					overhang_lines.insert(overhang_lines.end(), extra_lines.begin(), extra_lines.end());
				}
				break;
			}

			#pragma omp critical(precalculated_support_layers)
			{
				storage.support.supportLayers[layer_idx-dtt_roof].support_roof.add(overhang_outset);
			}

			last_overhang=overhang_outset;
			overhang_outset=overhang_outset_next;
		}

		if(!roof_enabled){
			Polygons polylines=ensureMaximumDistancePolyline(generateLines(overhang_outset,roof_enabled,0),support_line_width,1); // support_line_width to form a line here as otherwise most will be unsupported. Technically this violates branch distance, but not only is this the only reasonable choice, but it ensures consistant behaviour as some infill patterns generate each line segment as its own polyline part causing a similar line forming behaviour.
			if(polylines.pointCount()<=3){
				// add the outer wall to ensure it is correct supported instead
				polylines=ensureMaximumDistancePolyline(toPolylines(overhang_outset),connect_length,3);
			}
			overhang_lines=convertLinesToInternal(polylines,layer_idx);
		}
		else if (overhang_lines.empty()){ // nothing was added because the support roof is still large enough so now the area has to be converted to lines.
			overhang_lines=convertLinesToInternal(ensureMaximumDistancePolyline(generateLines(overhang_outset.offset(-5),roof_enabled,dtt_roof-1),connect_length,1),layer_idx-dtt_roof); //-5 to avoid rounding errors
		}


		if(int(dtt_roof)>layer_idx&&roof_enabled){ // reached buildplate

#pragma omp critical(precalculated_support_layers)
{
				storage.support.supportLayers[0].support_roof.add(overhang_outset);
}

		}
		else{ // normal trees have to be generated
			for(LineInformation line:overhang_lines){
				for(auto point_data:line){
					addToData(point_data,0,layer_idx-dtt_roof,mesh_config.support_roof_layers-dtt_roof,dtt_roof!=0);
				}
			}
		}
	}
	}

}


std::string getPolygonAsString(const Polygons& poly){ //todo remove as only for debugging relevant

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
		coord_t layer_nr ){

	for(std::map<SupportElement,Polygons>::iterator influence_iter = input.begin(); influence_iter != input.end(); influence_iter++){
			bool merged=false;

			AABB influence_aabb = input_aabb.count(influence_iter->first)?input_aabb.at(influence_iter->first):AABB(influence_iter->second);
			for(std::map<SupportElement,AABB>::iterator reduced_check_iter = reduced_new_layer_aabb.begin(); reduced_check_iter != reduced_new_layer_aabb.end(); reduced_check_iter++){

				AABB aabb = reduced_check_iter->second;
				if(aabb.hit(influence_aabb)) {

					bool merging_gracious_and_non_gracious = reduced_check_iter->first.to_model_gracious != influence_iter->first.to_model_gracious; // we do not want to merge a gracious with a non gracious area as bad placement could kill the hole subtree
					bool merging_to_bp=reduced_check_iter->first.to_buildplate&&influence_iter->first.to_buildplate; // get from main otherwise get from secondary

					coord_t increased_to_model_radius=0;
					size_t larger_to_model_ddt=0;

					if(!merging_to_bp){

						coord_t infl_radius=config.getRadius(influence_iter->first);
						coord_t redu_radius=config.getRadius(reduced_check_iter->first);
						if(reduced_check_iter->first.to_buildplate!=influence_iter->first.to_buildplate){ // get the real radius increase as the user does not care for the collision model.
							if(reduced_check_iter->first.to_buildplate){
								if(infl_radius<redu_radius){
									increased_to_model_radius=influence_iter->first.increased_to_model_radius + redu_radius-infl_radius;
								}
							}
							else{
								if(infl_radius>redu_radius){
									increased_to_model_radius=reduced_check_iter->first.increased_to_model_radius + infl_radius-redu_radius;
								}
							}
						}
						larger_to_model_ddt=std::max(influence_iter->first.distance_to_top,reduced_check_iter->first.distance_to_top);
					}

					// if a merge could place a stable branch on unstable ground, would be increasing the radius further than allowed to when merging to model and to_bp trees or would merge to model before it is known they will even been drawn the merge is skipped
					if(merging_gracious_and_non_gracious||increased_to_model_radius>config.max_to_model_radius_increase||(!merging_to_bp && larger_to_model_ddt<config.min_ddt_to_model&& !reduced_check_iter->first.supports_roof && !influence_iter->first.supports_roof)){
						continue;
					}

					Polygons relevant_infl;
					Polygons relevant_redu;

					if(merging_to_bp)
					{
						relevant_infl= main.count(influence_iter->first)?main.at(influence_iter->first):Polygons(); // influence_iter->first is a new element => not required to check if it was changed
						relevant_redu= insert_main.count(reduced_check_iter->first)?insert_main[reduced_check_iter->first]:(main.count(reduced_check_iter->first)?main.at(reduced_check_iter->first):Polygons());
					}
					else{
						relevant_infl= secondary.count(influence_iter->first)?secondary.at(influence_iter->first):Polygons();
						relevant_redu= insert_secondary.count(influence_iter->first)?insert_secondary[reduced_check_iter->first]:(secondary.count(reduced_check_iter->first)?secondary.at(reduced_check_iter->first):Polygons());
					}

					const bool red_bigger=config.getCollisionRadius(reduced_check_iter->first)>config.getCollisionRadius(influence_iter->first);
					std::pair<SupportElement,Polygons> smaller_rad = red_bigger ? std::pair<SupportElement,Polygons>(influence_iter->first,relevant_infl) : std::pair<SupportElement,Polygons>(reduced_check_iter->first,relevant_redu);
					std::pair<SupportElement,Polygons> bigger_rad =red_bigger ? std::pair<SupportElement,Polygons>(reduced_check_iter->first,relevant_redu) : std::pair<SupportElement,Polygons>(influence_iter->first,relevant_infl);
					const coord_t real_radius_delta=std::abs(config.getRadius(bigger_rad.first)-config.getRadius(smaller_rad.first));
					const coord_t smaller_collision_radius=config.getCollisionRadius(smaller_rad.first);

					if(bigger_rad.first.use_min_xy_dist&&!smaller_rad.first.use_min_xy_dist){ // the area of the bigger radius is used to ensure correct placement regarding the relevant avoidance, so if the use_min_radius would change an invalid area may be created
						continue;
					}

					// The idea is that the influence area with the smaller collision radius is increased by the radius difference.
					// If this area has any intersections with the influence area of the larger collision radius, a branch (of the larger collision radius) placed in it has already engulfed the branch of the smaller collision radius.
					// Because of this a merge may happen even if the influence areas (that represent possible center points of branches) do not intersect yet.
					// Remember that collision radius <= real radius as otherwise this assumption would be false.

					Polygons small_rad_increased_by_big_minus_small= safeOffsetInc(smaller_rad.second, real_radius_delta, volumes_.getCollision(smaller_collision_radius, layer_nr-1, false), 2*(config.xy_distance+smaller_collision_radius-3), 2*(config.xy_distance+smaller_collision_radius-3), 0);// -3 avoids possible rounding errors
					Polygons intersect= small_rad_increased_by_big_minus_small.intersection(bigger_rad.second);

					if(intersect.area()>1){ // dont use empty as a line is not empty, but for this use-case it very well may be (and will be one layer down as union does not keep lines)

						if(intersect.offset(-25).area()<=1) // check if the overlap is good enough. While 25 was guessed as enough, i did not have reason to change it
						{
							continue;
						}

						Point new_pos = reduced_check_iter->first.next_position;
						if(!intersect.inside(new_pos, true)){
							PolygonUtils::moveInside(intersect, new_pos);
						}

						if(increased_to_model_radius==0){
							increased_to_model_radius=std::max(reduced_check_iter->first.increased_to_model_radius,influence_iter->first.increased_to_model_radius);
						}
						SupportElement key(reduced_check_iter->first,influence_iter->first,layer_nr-1,new_pos,increased_to_model_radius,config);

						Polygons intersect_sec;
						if(merging_to_bp&&config.support_rests_on_model){

							Polygons sec_small=insert_secondary.count(smaller_rad.first)?insert_secondary[smaller_rad.first]:(secondary.count(smaller_rad.first)?secondary.at(smaller_rad.first):Polygons());
							Polygons sec_big=insert_secondary.count(bigger_rad.first)?insert_secondary[bigger_rad.first]:(secondary.count(bigger_rad.first)?secondary.at(bigger_rad.first):Polygons());
							Polygons small_rad_increased_by_big_minus_small_sec=safeOffsetInc(sec_small, real_radius_delta, volumes_.getCollision(smaller_collision_radius, layer_nr-1, false), 2*(config.xy_distance+smaller_collision_radius-3), 2*(config.xy_distance+smaller_collision_radius-3), 0);
							intersect_sec= small_rad_increased_by_big_minus_small_sec.intersection(sec_big);// if the one with the bigger radius with the lower radius removed overlaps we can merge
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
						Polygons merge = intersect.unionPolygons(intersect_sec).offset(config.getRadius(key), ClipperLib::jtRound).difference(volumes_.getCollision(0, layer_nr-1));

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



void TreeSupport::mergePolygonsDaQ(std::map<SupportElement,Polygons>& input,std::unordered_map<SupportElement,Polygons>& main,std::unordered_map<SupportElement,Polygons>& secondary,int layer_nr){
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


Polygons TreeSupport::safeOffsetInc(Polygons& me,coord_t distance,const Polygons& collision,coord_t safe_step_size,coord_t last_safe_step_size,size_t min_amount_offset)const{

	if(distance==0){
		return me.difference(collision).unionPolygons();
	}
	if(safe_step_size<0||last_safe_step_size<0){
		logError("Offset increase got negative distance!\n");
		return me.difference(collision).unionPolygons();
	}

	size_t counted=0;
	coord_t step_size=safe_step_size;
	size_t steps = distance>last_safe_step_size ? (distance-last_safe_step_size)/step_size:0;
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


void TreeSupport::increaseAreas(std::unordered_map<SupportElement,Polygons>& new_layer,
		std::map<SupportElement,Polygons>& new_layer_merge,
		std::unordered_map<SupportElement,Polygons>& new_layer_to_model,
		std::vector<std::pair<SupportElement*,Polygons*>>& new_layer_bypass_merge,
		const std::vector<std::pair<SupportElement*,Polygons*>>& last_layer,size_t layer_nr){

#pragma omp parallel for shared(new_layer,new_layer_merge,new_layer_to_model,last_layer) schedule(dynamic)
		for(long long unsigned int i=0;i<last_layer.size();i++){

			std::pair<SupportElement*,Polygons*> tup = last_layer[i];

			SupportElement elem(tup.first); // also increases ddt

			coord_t extra_over_speed=0;

			// it is assumed that the good areas from our new collision will overlap with the good areas of the old collision, but it is required to use 0 radius for the old collision as the collision may change on the layer below, causing the area to be possible in the first place
			// this may not be the most efficient way as a bigger collision would increase the safeStepSize, but it is a correct one
			Polygons wall_restriction=volumes_.getCollision(0, layer_nr,elem.use_min_xy_dist).intersection(volumes_.getCollision(config.getCollisionRadius(*tup.first), layer_nr-1,elem.use_min_xy_dist));//it is assumed that usually not more than num_threads areas are increased, meaning that calculating as needed, is faster than precalculating this area for all radiis.

			Polygons new_layer_data,new_layer_data_to_model,check_layer_data,increased;
			coord_t radius;
			const coord_t radius_lag=config.getRadius(elem)-config.getCollisionRadius(elem);
			const coord_t extra_slow_speed= radius_lag>config.support_line_width/5?std::min((config.maximum_move_distance-config.maximum_move_distance_slow)/2,std::max(config.support_line_width/5,radius_lag/10)):0; //if there is a noticeably collision radius lag => increase the slow movement speed

			std::function<bool(bool,bool,bool,bool,bool)> setPolygons=[&](bool slow,bool increase_radius,bool simplify,bool use_min_distance,bool move) { // this correctly sets the polygons defined above according to the parameters and returns if a new layer is possible
				SupportElement simulate_increase(elem);
				if(increase_radius){
					simulate_increase.effective_radius_height+=1;
				}
				radius = increase_radius? config.getCollisionRadius(simulate_increase) : config.getCollisionRadius(elem);

				coord_t extra_move_speed= std::min(config.getRadius(simulate_increase)-config.getRadius(*tup.first),config.maximum_move_distance_slow/2); // as the radius increases a move away (to the outside) from the circle happens. As it is not specified that the angle has to be the middle of a branch, it is here seen as the smaller angle of the outside of the resulting circle granting extra movement speed.
				if(extra_move_speed<0){
					logWarning("Tree Support: extraMoveSpeed<0. This implies that a branch is getting smaller while going down. This may cause problems.");
				}

				if(slow&&extra_move_speed<extra_slow_speed){ // move faster hoping to reduce the collision radius lag.
					extra_move_speed=extra_slow_speed;
				}

				extra_move_speed+=extra_over_speed;

				if(move){
					increased=safeOffsetInc(*tup.first->area, extra_move_speed + (slow?config.maximum_move_distance_slow:config.maximum_move_distance), wall_restriction,elem.use_min_xy_dist?config.xy_min_distance:config.xy_distance ,radius+(elem.use_min_xy_dist?config.xy_min_distance:config.xy_distance), 2); //offsetting in 2 steps makes our offsetted area rounder preventing (rounding) errors created by to pointy areas. May not be a problem anymore though.

					if(simplify){
						increased=increased.smooth(5);
						increased.simplify(15);
					}
				}
				else{ // if no movement is done the areas can be kept as at least one point (the one in the middle: result_on_layer) is still valid
					new_layer_data=*tup.first->area;
					new_layer_data_to_model=*tup.first->area;
					if(elem.to_buildplate){
						return !volumes_.getAvoidance(radius, layer_nr-1,slow , false,use_min_distance).inside(tup.first->result_on_layer, true);
					}
					else{
						if(elem.to_model_gracious){
							return !volumes_.getAvoidance(radius, layer_nr-1,slow , true,use_min_distance).inside(tup.first->result_on_layer, true);
						}
						else{
							return !volumes_.getCollision(radius, layer_nr-1,use_min_distance).inside(tup.first->result_on_layer, true);
						}
					}
				}
				new_layer_data= increased.difference(volumes_.getAvoidance(radius,layer_nr-1,slow,false,use_min_distance)).unionPolygons();
				if(config.support_rests_on_model){
					 new_layer_data_to_model= increased.difference(volumes_.getAvoidance(radius, layer_nr-1,slow , true,use_min_distance)).unionPolygons();
					 if(!elem.to_model_gracious){
						 if(new_layer_data_to_model.area()>=1){
							 elem.to_model_gracious=true;
							 logDebug("Corrected taint leading to a wrong non gracious value on layer %lld targeting %lld with radius %lld\n",layer_nr-1,elem.target_height,radius);
						 }
						 else{
							 new_layer_data_to_model = increased.difference(volumes_.getCollision(radius, layer_nr-1,use_min_distance)).unionPolygons();
						 }
					 }
				}
				if(new_layer_data.area()>1&& !elem.to_buildplate){ //mostly happening in the tip, but with merges ...
					elem.to_buildplate=true; // sometimes nodes that can reach the buildplate are marked as cant reach, tainting subtrees. this fixes this
					logDebug("Corrected taint leading to a wrong to model value on layer %lld targeting %lld with radius %lld\n",layer_nr-1,elem.target_height,radius);
				}
				check_layer_data=elem.to_buildplate ? new_layer_data : new_layer_data_to_model;

				if(increase_radius&&check_layer_data.area()>1){
					std::function<bool(bool,bool,bool)> nextStep=[&](bool slow_2,bool increase_radius_2,bool simplify_2) { // saves current status, and trys another step with the same settings to check if the change was valid
						Polygons new_layer_data_backup=Polygons(new_layer_data); // remember, setPolygons is by DEFINITION not non destructive
						Polygons check_layer_data_backup=Polygons(check_layer_data);
						Polygons new_layer_data_to_model_backup=Polygons(new_layer_data_to_model);
						size_t radius_backup=radius;

						if(!setPolygons(slow_2,increase_radius_2,simplify_2,use_min_distance,move)){ // we can not catch up further
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

					if(increase_bp_foot&&config.getRadius(elem)>=config.branch_radius){
						elem.elephant_foot_increases+=1;
						if(!nextStep(slow,false,simplify)){
							elem.elephant_foot_increases-=1;
						}
						else{
							radius=config.getCollisionRadius(elem);
						}
					}

					if(config.getRadius(elem) < config.increase_radius_until_radius&&elem.effective_radius_height<elem.distance_to_top){

						coord_t orig_ceil= volumes_.ceilRadius(config.getRadius(elem.effective_radius_height,elem.elephant_foot_increases),use_min_distance);
						bool increasing_radius=config.getRadius(elem.effective_radius_height)!=config.getRadius(elem.effective_radius_height+1);
						size_t max_effective_dtt = elem.effective_radius_height;
						for (coord_t i =1;volumes_.ceilRadius(config.getRadius(elem.effective_radius_height+i,elem.elephant_foot_increases),use_min_distance)==orig_ceil&&increasing_radius;i++){

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
			bool bypass=false;
			constexpr bool slow=true,increase_radius =true,simplify=true,use_min_radius=true,move=true;
			const bool reached_branch_radius=config.getRadius(elem)>=config.increase_radius_until_radius; // stable enough to risk not increasing the radius so the use_min_radius can go away.

			if(elem.distance_to_top<elem.dont_move_until&&setPolygons(slow,increase_radius,!simplify,!use_min_radius,!move)){
				add=true;
				bypass=true;
				elem.use_min_xy_dist=false;
				elem.result_on_layer=tup.first->result_on_layer;
			}
			else if(
						setPolygons(slow,increase_radius,simplify,!use_min_radius,move)||
						setPolygons(slow,!increase_radius,simplify,!use_min_radius,move)||
						setPolygons(!slow,increase_radius,simplify,!use_min_radius,move)||
						setPolygons(!slow,!increase_radius,simplify,!use_min_radius,move)
){
				add=true;
				elem.use_min_xy_dist=false;
				elem.dont_move_until=0;
			}
			else if(elem.use_min_xy_dist&&(

						(!reached_branch_radius&&setPolygons(slow,increase_radius,simplify,use_min_radius,move))||
						setPolygons(slow,!increase_radius,simplify,use_min_radius,move)||
						(!reached_branch_radius&&setPolygons(!slow,increase_radius,simplify,use_min_radius,move))||
						setPolygons(!slow,!increase_radius,simplify,use_min_radius,move))
			){
					add=true;
					if(elem.distance_to_top<config.tip_layers){ //do not bypass if the min distance already was held to long. As the merge does a safe offset, meaning the regular collision is subtracted from the offset, no invalid result can be created.
						bypass=true;
					}
					elem.dont_move_until=0;
			}
			else{

				if(!elem.to_buildplate&&(!elem.to_model_gracious ||( !tup.first->area->intersection(volumes_.getPlaceableAreas(radius, layer_nr)).empty()))) // we can use empty here as we dont need area inside the polygon i think
				{
					tup.first->result_on_layer=Point(-1,-1); // most down point has to be set somewhere else, as this enables the culling of to small branches
					continue; // it is normal that we wont be able to find a new area at some point in time if we wont be able to reach layer 0 aka have to connect with the model
				}
				logError("Influence area could not be increased! Data about the Influence area: Radius: %lld at layer: %lld NextTarget: %lld Distance to top: %lld Elephant foot increases %f  use_min_xy_dist %d to buildplate %d gracious %d\n",radius,layer_nr-1,elem.next_height,elem.distance_to_top,elem.elephant_foot_increases,elem.use_min_xy_dist,elem.to_buildplate,elem.to_model_gracious);

				extra_over_speed=config.maximum_move_distance/2;
				if(setPolygons(!slow,!increase_radius,!simplify,elem.use_min_xy_dist,move)){
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
				merge = merge_area_inc.difference(volumes_.getCollision(0, layer_nr-1,elem.use_min_xy_dist));
				#pragma omp critical(newLayer)
				{
					if(bypass){
						Polygons* new_area;
						if(config.support_rests_on_model){
							new_area=new Polygons(new_layer_data_to_model);
						}
						else{
							new_area=new Polygons(new_layer_data);
						}
						SupportElement* next= new SupportElement(elem,new_area);
						new_layer_bypass_merge.emplace_back(next,new Polygons(merge));
					}
					else{
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

}


void TreeSupport::createLayerPathing(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds){

	const double data_size_inverse = 1/double(move_bounds.size());
	double progress_total =PROGRESS_PRECALC_AVO+PROGRESS_PRECALC_COLL+PROGRESS_GENERATE_NODES;

	auto dur_inc=std::chrono::duration_values<std::chrono::nanoseconds>::zero();
	auto dur_merge=std::chrono::duration_values<std::chrono::nanoseconds>::zero();

	//Increase Influence Radius of each influence circle check for overlap and insert a new influence circle
	for (size_t layer_nr = move_bounds.size() - 1; layer_nr > 0; layer_nr--){

		std::map<SupportElement,Polygons> new_layer_merge; // merge maps are increased by effective radius to be able to ensure correct merge when 2 elements have a different radius, have to be ordered maps to ensure deterministic behavior
		std::unordered_map<SupportElement,Polygons> new_layer,new_layer_to_model;
		std::vector<std::pair<SupportElement*,Polygons*>> new_layer_bypass_merge; //other as the other maps of SupportElement these have the area already set, the .second are the equivalent of the new_layer_merge data like in move_bounds.
		auto ta = std::chrono::high_resolution_clock::now();

		std::vector<std::pair<SupportElement*,Polygons*>> last_layer;
		last_layer.insert(last_layer.begin(), move_bounds[layer_nr].begin(), move_bounds[layer_nr].end());

		increaseAreas(new_layer, new_layer_merge, new_layer_to_model,new_layer_bypass_merge, last_layer, layer_nr);


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

			if(elem.to_buildplate){
				continue;
			}
			Polygons* new_area= new Polygons(tup.second);
			SupportElement* next= new SupportElement(elem,new_area);
			move_bounds[layer_nr-1][next]=new Polygons(new_layer_merge[tup.first]);

			if(new_area->area()<1){
				logError("Insert Error of Influence area to model on layer %lld.\n",layer_nr-1);
			}
		}

		for(std::pair<SupportElement*,Polygons*> tup:new_layer_bypass_merge){

			move_bounds[layer_nr-1][tup.first]=tup.second;
		}

		progress_total+=data_size_inverse * PROGRESS_AREA_CALC;
		Progress::messageProgress(Progress::Stage::SUPPORT, progress_total*progress_multiplier+progress_offset,PROGRESS_TOTAL);
	}

    logDebug("Time spent with creating influence areas' subtaks: Increasing areas %lld ms merging areas: %lld ms\n",dur_inc.count()/1000000,dur_merge.count()/1000000);

}


void TreeSupport::setPointsOnAreas(const SupportElement* elem){

	if(elem->result_on_layer==Point(-1,-1)){
		logError("Uninitialised support element\n");
		return;
	}

	for(SupportElement* next_elem : elem->parents){

		if(next_elem->result_on_layer!=Point(-1,-1)) // if the value was set somewhere else it it kept. This happens when a branch tries not to move after being unable to create a roof.
		{
			continue;
		}

		Point from = elem->result_on_layer;
		if(!(next_elem->area->inside(from, true))){
			PolygonUtils::moveInside(*next_elem->area,from,0); // Move inside has edgecases (see tests) so DONT use Polygons.inside to confirm correct move, Error with dist 0 is <= 1 //
			//it is not required to check if how far this move moved a point as is can be larger than maximum_movement_speed. While this seems like a problem it may occur after merges or because the radius changed => which is free movement speed
		}

		coord_t allowed_movement=std::min(config.maximum_move_distance_slow/2,config.support_line_width/3)-vSize(from-elem->result_on_layer);
		if(allowed_movement>0&&elem->next_position==next_elem->next_position&&elem->next_height==next_elem->next_height){ // no merge happened on this layer so it can be attempted to move closer to the next target

			const size_t steps= std::max(coord_t(1), allowed_movement/config.getCollisionRadius(*next_elem));
			const coord_t move_per_step=allowed_movement/steps;
			for(size_t move_step = 0;move_step<steps;move_step++){
				Point vector_to_target= next_elem->next_position - from;
				if(vSize(vector_to_target)<move_per_step)
				{
					if(next_elem->area->inside(next_elem->next_position, true)){
						from=next_elem->next_position;
					}
					break;
				}
				else{
					double scale = (allowed_movement*1.0/vSize(vector_to_target));
					Point next_candidate=from+vector_to_target*scale;
					if(next_elem->area->inside(next_candidate, true)){
						from=next_candidate;
					}
					else{
						break;
					}
				}
			}

		}

		next_elem->result_on_layer=from;
		//no we do not call recursive because if we do out amount of layers is restricted by our stack size -.-'
	}

}

bool TreeSupport::setToModelContact (std::vector<std::map<SupportElement*,Polygons*>>& move_bounds, SupportElement* first_elem ,const size_t layer_nr){


	if(first_elem->to_model_gracious){

		SupportElement* check=first_elem;

		std::vector<SupportElement*> checked;
		size_t last_successfull_layer=layer_nr;
		bool set=false;
		for(size_t layer_check=layer_nr;check->next_height>=layer_check;layer_check++){
			if(!check->area->intersection(volumes_.getPlaceableAreas(config.getCollisionRadius(*check), layer_check)).empty()){
				set=true;
				last_successfull_layer=layer_check;
			}
			checked.emplace_back(check);
			if(check->parents.size()==1)
			{
				check=check->parents[0];
			}
			else{
				break; // reached merge point
			}
		}

		if(!set){
			if(SUPPORT_TREE_ONLY_GRACIOUS_TO_MODEL){
				logWarning("No valid placement found for to model gracious element on layer %lld: REMOVING BRANCH\n",layer_nr);

				for(size_t layer=layer_nr;layer<=first_elem->next_height;layer++){
					move_bounds[layer].erase(checked[layer-layer_nr]);
					delete checked[layer-layer_nr]->area;
					delete checked[layer-layer_nr];
				}
			}
			else{
				logWarning("No valid placement found for to model gracious element on layer %lld\n",layer_nr);
				first_elem->to_model_gracious=false;
				return setToModelContact(move_bounds, first_elem, layer_nr);
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
		{
			PolygonUtils::moveInside(*checked[last_successfull_layer-layer_nr]->area,best);
		}
		checked[last_successfull_layer-layer_nr]->result_on_layer=best;

		logDebug("Added gracious Support On Model on layer Point (%lld,%lld) my current layer is %lld\n",best.X,best.Y,last_successfull_layer);

		return last_successfull_layer!=layer_nr;


	}
	else{ // can not add graceful => just place it here and hope for the best
		Point best=first_elem->next_position;
		if(!first_elem->area->inside(best, true))
		{
			PolygonUtils::moveInside(*first_elem->area,best);
		}
		first_elem->result_on_layer=best;
		first_elem->to_model_gracious=false;
		logDebug("added NON gracious  Support On Model on layer Point (%lld,%lld) my current layer is %lld\n",best.X,best.Y,layer_nr);
		return false;
	}

}



void TreeSupport::createNodesFromArea(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds){

	for(std::pair<SupportElement*,Polygons*> init : move_bounds[0]){ // init points on layer 0
		Point p=init.first->next_position;
		if(!(init.first->area->inside(p, true)))
		{
			PolygonUtils::moveInside(*init.first->area,p,0 );
		}
		init.first->result_on_layer=p;
		setPointsOnAreas(init.first);
	}

	std::map<SupportElement,int> ignore;

	for (size_t layer_nr = 1; layer_nr < move_bounds.size(); layer_nr++){
		std::unordered_set<SupportElement*> remove;
		for(std::pair<SupportElement*,Polygons*> influence_area_tup : move_bounds[layer_nr]){

			bool removed=false;
			if(influence_area_tup.first->result_on_layer==Point(-1,-1)){
				if(influence_area_tup.first->to_buildplate||(!influence_area_tup.first->to_buildplate&&influence_area_tup.first->distance_to_top<config.min_ddt_to_model)){
					if(influence_area_tup.first->to_buildplate){
						logError("UNKNOWN POLYGON targeting (%lld,%lld) at target_height: %lld layer: %lld\n",influence_area_tup.first->target_position.X,influence_area_tup.first->target_position.Y,influence_area_tup.first->target_height,layer_nr);

					}
					remove.emplace(influence_area_tup.first); // we dont need to remove yet the parents as they will have a lower dtt and also no result_on_layer set
					removed=true;
					for(SupportElement* elem:influence_area_tup.first->parents){
						elem->result_on_layer=Point(-1,-1); // as not sucessfull roof causes a no move the parents need to be invalidated to be sure they will be removed later
					}
					continue;
				}
				else{// we need to connect with the model
					removed=setToModelContact(move_bounds, influence_area_tup.first, layer_nr);
					if(removed)
					{
						remove.emplace(influence_area_tup.first);
					}
				}
			}

			if(!removed)
			{
				setPointsOnAreas(influence_area_tup.first);//element is valid now setting points in the layer above
			}
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

bool fuzzyInside(Polygons& area,Point p,coord_t fuzzyness){
	Point from =p;
	PolygonUtils::moveInside(area, from,0);
	return area.inside(p, true) || vSize(p-from)<fuzzyness;
}

Polygons getPolygonsPartWithPointInside(Polygons& poly,Point inside){
	for(PolygonsPart part: poly.splitIntoParts(true)){
		if(fuzzyInside(part,inside,50)){
			return part;
		}
	}
	logWarning("Could not find area with middle point in it");
	return Polygons();
}

void TreeSupport::drawAreas(std::vector<std::map<SupportElement*,Polygons*>>& move_bounds,SliceDataStorage &storage){
	//we do most of the things as in the original draw circle but we can improve a few things as we already calculated maximal outer bounds for our tree wall we can now cut areas out of them elliminating the need for a slow difference, using a much faster intersect instead
	//also we can add the z- bottom distance to precalc
	double progress_total =PROGRESS_PRECALC_AVO+PROGRESS_PRECALC_COLL+PROGRESS_GENERATE_NODES+PROGRESS_AREA_CALC;

	const Settings &mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	const size_t wall_count = mesh_group_settings.get<size_t>("support_wall_count");
	Polygon branch_circle; //Pre-generate a circle with correct diameter so that we don't have to recompute those (co)sines every time.
	for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++) {
		const AngleRadians angle = static_cast<double>(i) / CIRCLE_RESOLUTION * TAU;
		branch_circle.emplace_back(cos(angle) * config.branch_radius, sin(angle) * config.branch_radius);
	}
	const size_t z_distance_bottom_layers = config.z_distance_bottom_layers;
	const std::function<SupportElement*(SupportElement*)> getBiggestRadiusParrent=[&](const SupportElement* elem) {
		SupportElement* ret=nullptr;
		for(SupportElement* el2:elem->parents){
			const coord_t rad=config.getRadius(*el2);
			if(ret==nullptr||rad>config.getRadius(*ret))
			{
				ret=el2;
			}
		}
		return  ret;
	};
	std::vector<Polygons> support_layer_storage(move_bounds.size());
	std::map<SupportElement*,std::pair<SupportElement*,Polygons*>> inverese_tree_order; // in our tree we can only access the parents. We inverse this to be able to access the children.
	std::vector<std::pair<size_t,std::pair<SupportElement*,Polygons*>>> linear_data; // All SupportElements are put into a layer independent storage (the size_t is the layer number) to improve parallelization. Was added at a point in time where this function had performance issues.
																					// These were fixed by creating less initial points, but i do not see a good reason to remove a working performance optimization.
	for (size_t layer_nr = 0; layer_nr < move_bounds.size();layer_nr++) {
		for(std::pair<SupportElement*,Polygons*> tup: move_bounds[layer_nr]){

			if((layer_nr>0&&((!inverese_tree_order.count(tup.first)&&tup.first->target_height==layer_nr)||(inverese_tree_order.count(tup.first)&&inverese_tree_order[tup.first].first->result_on_layer==Point(-1,-1))))) // we either come from nowhere at the final layer or we had invalid parents 2. should never happen but just to be sure
			{
				continue;
			}

			for(SupportElement* par : tup.first->parents){
				if(par->result_on_layer==Point(-1,-1))
				{
					continue;
				}
				inverese_tree_order.emplace(par,tup);
			}
			linear_data.emplace_back(layer_nr,tup);
		}
	}

	std::vector<Polygons> linear_inserts(linear_data.size());
	std::vector<std::vector<std::pair<size_t,Polygons>>> dropped_down_areas(linear_data.size());
	const size_t progress_inserts_check_interval=linear_data.size()/10;
	// parallel iterating over all elements
#pragma omp parallel for shared(linear_inserts, linear_data, inverese_tree_order)
	for(coord_t i=0;i<static_cast<coord_t>(linear_data.size());i++){
		const std::pair<SupportElement*,Polygons*> tup= linear_data[i].second;
		double scale = config.getRadius(*tup.first)/(1.0*config.branch_radius);// std::max(config.getRadius(*tup.first),config.support_line_width)


		coord_t effective_radius=scale*config.branch_radius;
		std::vector<std::pair<Point,coord_t>> movement_directions{std::pair<Point,coord_t>(Point(0,0),effective_radius)};
		SupportElement* child_elem =  inverese_tree_order.count(tup.first)?inverese_tree_order.at(tup.first).first:nullptr;
		if(child_elem!=nullptr){
			Point movement =(child_elem->result_on_layer - tup.first->result_on_layer);
			if(tup.first->next_height==linear_data[i].first&&movement.X+movement.Y){ // if merge: limit move distance to avoid excessive ovalization
				coord_t radius_difference=  std::abs(config.getRadius(*tup.first)-config.getRadius(*child_elem));
				movement=movement*(vSize(movement)-radius_difference>0?vSize(movement)-radius_difference:0)/vSize(movement);
			}
			movement_directions.emplace_back(movement,effective_radius);
		}
		bool parent_uses_min=false;
		for(SupportElement* parrent:tup.first->parents){
			Point movement =( parrent->result_on_layer - tup.first->result_on_layer);
			if(tup.first->next_position==linear_data[i].first&&movement.X+movement.Y){ // if merge: limit move distance to avoid excessive ovalization
				coord_t radiusDifference=  std::abs(config.getRadius(*tup.first)-config.getRadius(*parrent));
				movement=movement*(vSize(movement)-radiusDifference>0?vSize(movement)-radiusDifference:0)/vSize(movement);

			}
			movement_directions.emplace_back(movement,std::max(config.getRadius(*parrent),config.support_line_width));
			parent_uses_min|=parrent->use_min_xy_dist;
		}

		coord_t max_speed=0;
		std::function<Polygons(coord_t)> generateArea=[&](coord_t offset){

			Polygons poly;

			for(std::pair<Point,coord_t> movement:movement_directions){
						double used_scale=(movement.second+offset)/(1.0*config.branch_radius);
						Point center_position = tup.first->result_on_layer+movement.first/2;
						max_speed=std::max(max_speed,vSize(movement.first));
						const double moveX = movement.first.X/(used_scale*config.branch_radius);
						const double moveY = movement.first.Y/(used_scale*config.branch_radius);
						const double vsize_inv=0.5/(0.01+std::sqrt(moveX*moveX+moveY*moveY));

						double matrix[] = {
								used_scale* (1 + moveX*moveX*vsize_inv),
								used_scale* (0 + moveX*moveY*vsize_inv),
								used_scale* (0 + moveX*moveY*vsize_inv),
								used_scale* (1 + moveY*moveY*vsize_inv),
						};
						Polygon circle;
						for (Point vertex : branch_circle) {
							vertex = Point(
									matrix[0]*vertex.X + matrix[1]*vertex.Y,
									matrix[2]*vertex.X + matrix[3]*vertex.Y);
							circle.add(center_position + vertex);
						}
						poly=poly.unionPolygons(circle.offset(0));//TODO UNION behaved different than add and union !!! WHY ????? This would mean collision and avoidance etc may be wrong ... FML
					}
			return poly;
		};

		Polygons poly=generateArea(0);
		bool fast_relative_movement=max_speed>effective_radius*0.75;

		const Polygons child = inverese_tree_order.count(tup.first)?*inverese_tree_order.at(tup.first).second:Polygons();
		if(fast_relative_movement||parent_uses_min||tup.first->distance_to_top==0){ //dtt 0 collision is not subtracted when inserting as it is better done here.
			linear_inserts[i]=poly.difference(volumes_.getCollision(0, linear_data[i].first,parent_uses_min||tup.first->use_min_xy_dist)).unionPolygons();// this is slower by about 25%
		}
		else{
			linear_inserts[i]=tup.second->unionPolygons(child).intersection(poly).unionPolygons();
		}
		if(fast_relative_movement||config.getRadius(*tup.first)-config.getCollisionRadius(*tup.first)>config.support_line_width){
			Polygons nozzle_path=linear_inserts[i].offset(-config.support_line_width/2); // simulate the path the nozzle will take on the outermost wall
			if(nozzle_path.splitIntoParts(false).size()>1){ // if multiple parts exist, the outer line will not go all around the support, part potentially causing support material to be printed mid air

				poly=generateArea(config.support_line_width/2); // Just try to make the area a tiny bit larger.
				linear_inserts[i]=poly.difference(volumes_.getCollision(0, linear_data[i].first,parent_uses_min||tup.first->use_min_xy_dist)).unionPolygons();
				nozzle_path=linear_inserts[i].offset(-config.support_line_width/2);
				if(nozzle_path.splitIntoParts(false).size()>1){ // if larger areas do not fix the problem, the area is increased and all parts that do not contain the center point are removed hoping for the best

					poly=generateArea(-config.getRadius(0)+config.support_line_width/2);
					poly=poly.difference(volumes_.getCollision(config.getRadius(0), linear_data[i].first,parent_uses_min||tup.first->use_min_xy_dist));
					Polygons polygons_with_correnct_center;
					for(PolygonsPart part: poly.splitIntoParts(true)){
						if( part.inside(tup.first->result_on_layer, true) ){
							polygons_with_correnct_center=polygons_with_correnct_center.unionPolygons(part);
						}
						else{
							Point from =tup.first->result_on_layer;
							PolygonUtils::moveInside(part, from,0);
							if(vSize(tup.first->result_on_layer-from)<25){// try a fuzzy inside as sometimes the point should be on the border, but is not because of rounding errors...
								polygons_with_correnct_center=polygons_with_correnct_center.unionPolygons(part);
							}
						}
					}
					poly =polygons_with_correnct_center.offset(config.getRadius(0));
					linear_inserts[i]=poly.difference(volumes_.getCollision(0, linear_data[i].first,parent_uses_min||tup.first->use_min_xy_dist)).unionPolygons();
				}
			}
		}

		bool non_gracious_model_contact=!tup.first->to_model_gracious && !inverese_tree_order.count(tup.first); // if a element has no child, it connects to whatever is below as no support further down for it will exist.
		if(non_gracious_model_contact){
			Polygons rest_support= linear_inserts[i];
			size_t counter=1;
			while(rest_support.area()>1&&counter<linear_data[i].first){
				rest_support=rest_support.difference(volumes_.getCollision(0, linear_data[i].first-counter));
				dropped_down_areas[i].emplace_back(linear_data[i].first-counter,rest_support);
				counter++;
			}
		}

		if(i%progress_inserts_check_interval==0){
#pragma omp critical(progress)
			{
				progress_total+=PROGRESS_DRAW_AREAS/20; //half the amount of progress in done in this loop and only 10 samples are reported
				Progress::messageProgress(Progress::Stage::SUPPORT,progress_total*progress_multiplier+progress_offset ,PROGRESS_TOTAL);
			}
		}

	}

	// single threaded combining all elements to the right layers. ONLY COPYS DATA!
	for(coord_t i=0;i<static_cast<coord_t>(linear_data.size());i++){
		support_layer_storage[linear_data[i].first].add(linear_inserts[i]);
		for(auto pair:dropped_down_areas[i]){
			support_layer_storage[pair.first].add(pair.second);
		}
	}
	progress_total=PROGRESS_PRECALC_AVO+PROGRESS_PRECALC_COLL+PROGRESS_GENERATE_NODES+PROGRESS_AREA_CALC+PROGRESS_DRAW_AREAS/2;
	linear_inserts.clear();
	linear_data.clear();

	//Iterate over the generated circles in parallel and clean them up. Also add support floor.
#pragma omp parallel for shared(support_layer_storage, storage) schedule(dynamic)
	for (coord_t layer_nr = 0; layer_nr < static_cast<coord_t>(support_layer_storage.size());layer_nr++) {
			support_layer_storage[layer_nr] = support_layer_storage[layer_nr].unionPolygons().smooth(50);
			storage.support.supportLayers[layer_nr].support_roof = storage.support.supportLayers[layer_nr].support_roof.unionPolygons();
			support_layer_storage[layer_nr] = support_layer_storage[layer_nr].difference(storage.support.supportLayers[layer_nr].support_roof);
			support_layer_storage[layer_nr].simplify(30,10);
			//Subtract support floors.
			if (config.support_bottom_layers>0) { // support bottom magic from the old drawCircle.
				Polygons &floor_layer =	storage.support.supportLayers[layer_nr].support_bottom;
				for (size_t layers_below = 0;
						layers_below < config.support_bottom_layers; layers_below +=
								config.performance_interface_skip_layers) {
					const size_t sample_layer = static_cast<size_t>(std::max(0,
							static_cast<int>(layer_nr)
									- static_cast<int>(layers_below)
									- static_cast<int>(z_distance_bottom_layers)));
					constexpr bool no_support = false;
					constexpr bool no_prime_tower = false;
					floor_layer.add(
							support_layer_storage[layer_nr].intersection(
									storage.getLayerOutlines(sample_layer,
											no_support, no_prime_tower)));
				}
				{ //One additional sample at the complete bottom height.
					const size_t sample_layer = static_cast<size_t>(std::max(0,
							static_cast<int>(layer_nr)
									- static_cast<int>(config.support_bottom_layers)
									- static_cast<int>(z_distance_bottom_layers)));
					constexpr bool no_support = false;
					constexpr bool no_prime_tower = false;
					floor_layer.add(
							support_layer_storage[layer_nr].intersection(
									storage.getLayerOutlines(sample_layer,
											no_support, no_prime_tower)));
				}
				floor_layer.unionPolygons();
				support_layer_storage[layer_nr] = support_layer_storage[layer_nr].difference(floor_layer.offset(10)); //Subtract the support floor from the normal support.
			}

		for (PolygonsPart part : support_layer_storage[layer_nr].splitIntoParts(true) ) //Convert every part into a PolygonsPart for the support.
		{
			PolygonsPart outline;
			outline.add(part);
			storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(
					outline, config.support_line_width, wall_count);
		}

#pragma omp critical(progress)
	{
		progress_total+=PROGRESS_DRAW_AREAS/(2*support_layer_storage.size());
		Progress::messageProgress(Progress::Stage::SUPPORT,progress_total*progress_multiplier+progress_offset ,PROGRESS_TOTAL);
	}

	#pragma omp critical (support_max_layer_nr)
			{
				if (!storage.support.supportLayers[layer_nr].support_infill_parts.empty()
						|| !storage.support.supportLayers[layer_nr].support_roof.empty())
				{
					storage.support.layer_nr_max_filled_layer = std::max(
							storage.support.layer_nr_max_filled_layer,
							static_cast<int>(layer_nr));
				}
			}
		}

}





ModelVolumes::ModelVolumes(const SliceDataStorage& storage,const coord_t max_move,const coord_t max_move_slow, size_t current_mesh_idx,double progress_multiplier,double progress_offset, const std::vector<Polygons>& additional_excluded_areas):
		max_move_{std::max(max_move-2,coord_t(0))}, max_move_slow_{std::max(max_move_slow-2,coord_t(0))},progress_multiplier{progress_multiplier},progress_offset{progress_offset},machine_border_ { calculateMachineBorderCollision(storage.getMachineBorder()) } // -2 to avoid rounding errors
{

	anti_overhang_= std::vector<Polygons>(storage.support.supportLayers.size(),Polygons());
	std::unordered_map<size_t,size_t> mesh_to_layeroutline_idx;
	min_maximum_deviation_=std::numeric_limits<coord_t>::max();
	min_maximum_resolution_=std::numeric_limits<coord_t>::max();
	support_rests_on_model=false;
	for(size_t mesh_idx=0;mesh_idx<storage.meshes.size();mesh_idx++){
		SliceMeshStorage mesh = storage.meshes[mesh_idx];
		bool added=false;
		for(size_t idx =0;idx<layer_outlines_.size();idx++){
			if(checkSettingsEquality(layer_outlines_[idx].first, mesh.settings)){
				added=true;
				mesh_to_layeroutline_idx[mesh_idx]=idx;
			}
		}
		if(!added){
			mesh_to_layeroutline_idx[mesh_idx]=layer_outlines_.size();
			layer_outlines_.emplace_back(mesh.settings,std::vector<Polygons>(storage.support.supportLayers.size(),Polygons()));
		}

	}

	for(auto data_pair: layer_outlines_){
		support_rests_on_model|=data_pair.first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
		min_maximum_deviation_=std::min(min_maximum_deviation_,data_pair.first.get<coord_t>("meshfix_maximum_deviation"));
		min_maximum_resolution_=std::min(min_maximum_resolution_,data_pair.first.get<coord_t>("meshfix_maximum_resolution"));
	}

	min_maximum_deviation_=std::min(coord_t(SUPPORT_TREE_MAX_DEVIATION),min_maximum_deviation_);
	current_outline_idx=mesh_to_layeroutline_idx[current_mesh_idx];

	if(layer_outlines_[current_outline_idx].first.get<SupportDistPriority>("support_xy_overrides_z")==SupportDistPriority::Z_OVERRIDES_XY){
		current_min_xy_dist=std::max(layer_outlines_[current_outline_idx].first.get<coord_t>("support_xy_distance_overhang"),coord_t(100));
		current_min_xy_dist_delta=std::max(layer_outlines_[current_outline_idx].first.get<coord_t>("support_xy_distance"),coord_t(100))-current_min_xy_dist;
	}
	else
	{
		current_min_xy_dist=layer_outlines_[current_outline_idx].first.get<coord_t>("support_xy_distance");
		current_min_xy_dist_delta=0;
	}

	for(size_t mesh_idx=0;mesh_idx<storage.meshes.size();mesh_idx++){
		SliceMeshStorage mesh = storage.meshes[mesh_idx];
		#pragma omp parallel for
		for(LayerIndex layer_idx=0;layer_idx<coord_t(layer_outlines_[mesh_to_layeroutline_idx[mesh_idx]].second.size());layer_idx++){
			if(mesh.layer_nr_max_filled_layer<layer_idx){
				continue; //cant break as omp for loop wont allow it
			}
			Polygons outline=extractOutlineFromMesh(mesh, layer_idx);
			layer_outlines_[mesh_to_layeroutline_idx[mesh_idx]].second[layer_idx].add(outline);
		}

	}
#pragma omp parallel for
	for(LayerIndex layer_idx=0;layer_idx<coord_t(anti_overhang_.size());layer_idx++){
		if(layer_idx<coord_t(additional_excluded_areas.size())){
			anti_overhang_[layer_idx].add(additional_excluded_areas[layer_idx]);
		}
		if(SUPPORT_TREE_AVOID_SUPPORT_BLOCKER)
		{
			anti_overhang_[layer_idx].add(storage.support.supportLayers[layer_idx].anti_overhang);
		}
		if(storage.primeTower.enabled){
			anti_overhang_[layer_idx].add(layer_idx == 0 ? storage.primeTower.outer_poly_first_layer : storage.primeTower.outer_poly);
		}
		anti_overhang_[layer_idx]=anti_overhang_[layer_idx].unionPolygons();

	}
	for(size_t idx =0;idx<layer_outlines_.size();idx++){
#pragma omp parallel for
		for(LayerIndex layer_idx=0;layer_idx<coord_t(anti_overhang_.size());layer_idx++){
			layer_outlines_[idx].second[layer_idx]=layer_outlines_[idx].second[layer_idx].unionPolygons();
		}
	}

}


void ModelVolumes::precalculate(coord_t max_layer){

	auto t_start = std::chrono::high_resolution_clock::now();

	precalculated=true;
	TreeSupport::TreeSupportSettings config(layer_outlines_[current_outline_idx].first);

	std::function<LayerIndex(coord_t,coord_t)> getRequiredHeight=[&](coord_t radius,coord_t radius_before) {
		LayerIndex max_required_layer=max_layer;
		if(radius>config.branch_radius){
			if(config.diameter_angle_scale_factor>0){

				double layers_handled_by_rad_before=std::max(((radius_before-current_min_xy_dist_delta)-config.branch_radius)/(config.branch_radius*config.diameter_angle_scale_factor),0.0);
				max_required_layer-=layers_handled_by_rad_before-2; // -2 to avoid off by one problems and a few layers more do not matter performance wise
			}
		}
		if(max_required_layer<config.layer_start_bp_radius){
			max_required_layer=config.layer_start_bp_radius;
		}
		if(max_required_layer>max_layer){
			max_required_layer=max_layer;
		}

		return max_required_layer;
	};
	coord_t last_min_radius=0;
	coord_t last_radius=0;
	for(size_t dtt=0;dtt<config.tip_layers;dtt++){
		if(ceilRadius(last_radius+1)!=ceilRadius(config.getRadius(dtt)+current_min_xy_dist_delta)&&ceilRadius(last_min_radius+1)!=ceilRadius(config.getRadius(dtt)))
		{
			ignorable_radii_.emplace(dtt);
		}
		last_min_radius=config.getRadius(dtt);
		last_radius=ceilRadius(last_min_radius+current_min_xy_dist_delta);
	}

	std::deque<RadiusLayerPair> relevant_avoidance_radiis;
	const coord_t max_radius=std::max(ceilRadius( config.branch_radius * (max_layer+1) * config.diameter_angle_scale_factor+config.branch_radius),ceilRadius(config.bp_radius));
	relevant_avoidance_radiis.emplace_back(RadiusLayerPair(ceilRadius(1),getRequiredHeight(ceilRadius(1),ceilRadius(1))));
	for (coord_t radius=ceilRadius(ceilRadius(1)+1);radius<= max_radius;radius=ceilRadius(radius+1)){

		LayerIndex max_rel_layer_idx=getRequiredHeight(radius,relevant_avoidance_radiis.back().first);
		relevant_avoidance_radiis.emplace_back(RadiusLayerPair(radius,max_rel_layer_idx));
	}

	std::deque<RadiusLayerPair> relevant_collision_radiis;
	relevant_collision_radiis.emplace_back(RadiusLayerPair(0,max_layer));
	std::unordered_map<coord_t,LayerIndex> col_radiis;
	for(RadiusLayerPair rlp:relevant_avoidance_radiis){

		if(!col_radiis.count(ceilRadius(rlp.first+current_min_xy_dist_delta))){

			col_radiis[ceilRadius(rlp.first+current_min_xy_dist_delta)]=rlp.second;
		}

		if(!col_radiis.count(rlp.first)){
			col_radiis[rlp.first]=rlp.second;
		}
	}
	relevant_collision_radiis.insert(relevant_collision_radiis.end(), col_radiis.begin(), col_radiis.end());

	calculateCollision(relevant_collision_radiis);
	auto t_coll = std::chrono::high_resolution_clock::now();
#pragma omp parallel // avoidance does NOT include a parallel block to enable working on multiple calculations at the same time
{
	calculateAvoidance(relevant_avoidance_radiis);
	if(support_rests_on_model){
		calculatePlaceables(relevant_avoidance_radiis);
		calculateAvoidanceToModel(relevant_avoidance_radiis);
	}
}
	auto t_end = std::chrono::high_resolution_clock::now();
	auto dur_col = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_coll-t_start ).count();
	auto dur_avo = 0.001* std::chrono::duration_cast<std::chrono::microseconds>( t_end-t_coll ).count();
	logDebug("Precalc collision took %.3lf ms avoidance took %.3lf ms\n",dur_col,dur_avo);
}

const Polygons& ModelVolumes::getCollision(coord_t orig_radius,LayerIndex layer_idx,bool min_xy_dist) {
	coord_t radius =orig_radius;
	std::optional<std::reference_wrapper<const Polygons>> result;
	if(!min_xy_dist){
		radius+=current_min_xy_dist_delta;
	}
	radius=ceilRadius(radius);
	RadiusLayerPair key { radius, layer_idx };

	#pragma omp critical(collision_cache_)
	{
		result= getArea(collision_cache_, key);
	}
	if(result){
		return result.value().get();
	}
	if(precalculated){
		logWarning("Had to calculate collision at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n",key.first,key.second);
	}
	calculateCollision(key);
	return getCollision(orig_radius, layer_idx,min_xy_dist);
}

const Polygons& ModelVolumes::getAvoidance(coord_t orig_radius, LayerIndex layer_idx,bool slow,bool to_model,bool min_xy_dist) {
	coord_t radius =orig_radius;

	std::optional<std::reference_wrapper<const Polygons>> result;

	if(!min_xy_dist){
		radius+=current_min_xy_dist_delta;
	}
	radius=ceilRadius(radius);
	const RadiusLayerPair key { radius, layer_idx };

	std::unordered_map<RadiusLayerPair, Polygons>* cache_ptr;
	if(!to_model&&!slow){
		cache_ptr=&avoidance_cache_;
	}
	else if(!to_model&&slow){
		cache_ptr=&avoidance_cache_slow_;
	}
	else if(to_model&&!slow){
		cache_ptr=&avoidance_cache_to_model_;
	}
	else{
		cache_ptr=&avoidance_cache_to_model_slow_;
	}


	if(to_model){
#pragma omp critical(avoidance_cache_to_model_)
		{
			result= getArea(*cache_ptr, key);
		}
		if(result){
			return result.value().get();
		}
		if(precalculated){
			logWarning("Had to calculate Avoidance to model at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n",key.first,key.second);
		}
		calculateAvoidanceToModel(key);

	}
	else{
#pragma omp critical(avoidance_cache_)
		{
			result= getArea(*cache_ptr, key);
		}
		if(result){
			return result.value().get();
		}
		if(precalculated){
			logWarning("Had to calculate Avoidance at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n",key.first,key.second);
		}
		calculateAvoidance(key);

	}
	return getAvoidance(orig_radius, layer_idx, slow, to_model, min_xy_dist); //retrive failed and correct result was calculated. Now it has to be retrived.

}

const Polygons& ModelVolumes::getPlaceableAreas(coord_t radius,LayerIndex layer_idx) {

	std::optional<std::reference_wrapper<const Polygons>> result;
	radius=ceilRadius(radius);
	RadiusLayerPair key { radius, layer_idx };

	#pragma omp critical(placeable_areas_cache_)
	{
		result= getArea(placeable_areas_cache_, key);
	}
	if(result){
		return result.value().get();
	}
	if(precalculated){
		logWarning("Had to calculate Placeable Areas at radius %lld and layer %lld, but precalculate was called. Performance may suffer!\n",radius,layer_idx);
	}
	if(radius!=0){
		#pragma omp critical(calculatePlaceables)
		{
			calculatePlaceables(key);
		}
	}
	else{
		getCollision(0, layer_idx);
	}
	return getPlaceableAreas(radius, layer_idx);
}

coord_t ModelVolumes::ceilRadius(coord_t radius,bool min_xy_dist) const {
	if(!min_xy_dist){
		radius+=current_min_xy_dist_delta;
	}
	return ceilRadius(radius);
}

bool ModelVolumes::checkSettingsEquality(const Settings& me, const Settings& other)const{

	return TreeSupport::TreeSupportSettings(me).hasSameAvoidanceSettings(TreeSupport::TreeSupportSettings(other));
}


Polygons ModelVolumes::extractOutlineFromMesh(const SliceMeshStorage& mesh, LayerIndex layer_idx)const{
	constexpr bool external_polys_only=false;
	Polygons total;

	// similar to SliceDataStorage.getLayerOutlines but only for one mesh instead of for everyone

	if (mesh.settings.get<bool>("infill_mesh") || mesh.settings.get<bool>("anti_overhang_mesh"))
	{
		return Polygons();
	}
	const SliceLayer& layer = mesh.layers[layer_idx];

	layer.getOutlines(total, external_polys_only);
	if (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
	{
		total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
	}
	coord_t maximum_resolution = mesh.settings.get<coord_t>("meshfix_maximum_resolution");
	coord_t maximum_deviation =  mesh.settings.get<coord_t>("meshfix_maximum_deviation");
	total.simplify(maximum_resolution, maximum_deviation);
	return total;
}

LayerIndex ModelVolumes::getMaxCalculatedLayer(coord_t radius,const std::unordered_map<RadiusLayerPair, Polygons>& map ) const {
	coord_t max_layer=0;
	coord_t significant_digits =1;
	bool found=false;
	while(map.count(RadiusLayerPair(radius,(significant_digits-1)*10))){
		significant_digits++;
	}

	for(;significant_digits>=0;significant_digits--){
		for(coord_t layer=max_layer;map.count(RadiusLayerPair(radius,layer)); layer+=std::max(10*significant_digits,coord_t(1))){
			max_layer=layer;
			found=true;
		}
	}


	return found?max_layer:-1;
}


void ModelVolumes::calculateCollision( std::deque<RadiusLayerPair> keys ){ //slow
	#pragma omp parallel for schedule(static,1)
	for(long long unsigned int i=0;i<keys.size();i++){
			coord_t radius = keys[i].first;
			RadiusLayerPair key(radius,0);
			std::unordered_map<RadiusLayerPair,Polygons> data_outer;
			std::unordered_map<RadiusLayerPair,Polygons> data_placeable_outer;
			for(size_t outline_idx=0;outline_idx<layer_outlines_.size();outline_idx++){
				std::unordered_map<RadiusLayerPair,Polygons> data;
				std::unordered_map<RadiusLayerPair,Polygons> data_placeable;

				const coord_t layer_height = layer_outlines_[outline_idx].first.get<coord_t>("layer_height");
				const bool support_rests_on_this_model =layer_outlines_[outline_idx].first.get<ESupportType>("support_type") == ESupportType::EVERYWHERE;
				const coord_t z_distance_bottom=support_rests_on_this_model?layer_outlines_[outline_idx].first.get<coord_t>("support_bottom_distance"):layer_outlines_[outline_idx].first.get<coord_t>("support_z_distance");// if support may only rest on build plate the UI removes support_bottom_distance so the regular support_z_distance is used
				const size_t z_distance_bottom_layers = round_up_divide(z_distance_bottom,layer_height);
				const coord_t z_distance_top_layers= round_up_divide(layer_outlines_[outline_idx].first.get<coord_t>("support_top_distance"),layer_height);

				const size_t max_required_layer = keys[i].second+std::max(coord_t(1),z_distance_top_layers);
				const coord_t xy_distance=TreeSupport::TreeSupportSettings(layer_outlines_[outline_idx].first).hasSameInfluenceAreaSettings(layer_outlines_[current_outline_idx].first)?
						current_min_xy_dist:layer_outlines_[outline_idx].first.get<coord_t>("support_xy_distance");
				// technically this causes collision for the normal xy_distance to be larger by current_min_xy_dist_delta for all not currently processing meshes as this delta will be added at request time.
				// avoiding this would require saving each collision for each outline_idx separately.
				// and later for each avoidance... But avoidance calculation has to be for the whole scene and can NOT be done for each outline_idx separately and combined later.
				// so avoiding this inaccuracy seems infeasible as it would require 2x the avoidance calculations => 0.5x the performance.
				coord_t min_layer_bottom;
#pragma omp critical(collision_cache_)
				{
					min_layer_bottom=getMaxCalculatedLayer(radius,collision_cache_)-z_distance_bottom_layers;
				}

				if(min_layer_bottom<0){
					min_layer_bottom=0;
				}
				for(size_t layer=min_layer_bottom;layer<=max_required_layer;layer++){
					key.second=layer;
					Polygons collision_areas=machine_border_;
					if (layer < layer_outlines_[outline_idx].second.size()) {
						collision_areas.add(layer_outlines_[outline_idx].second[layer]);
					}
					collision_areas = collision_areas.offset(radius+xy_distance); //jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
					data[key].add(collision_areas); // if a key does not exist when it is accessed it is added!

				}


				for(coord_t layer=static_cast<coord_t>(max_required_layer);layer>=min_layer_bottom;layer--){ // have to do signed as we are counting down
					key.second=layer;
					for(size_t layer_offset=1;layer_offset<=z_distance_bottom_layers&&layer-coord_t(layer_offset)>min_layer_bottom;layer_offset++){
						data[key].add(data[RadiusLayerPair(radius,layer-layer_offset)]);
					}
					if(support_rests_on_this_model&&radius==0&&layer<coord_t(1+ keys[i].second)){
						//key.second=layer+1;
						data[key]=data[key].unionPolygons();
						Polygons above=data[RadiusLayerPair(radius,layer+1)];
						if(anti_overhang_.size()>size_t(layer+1)){
							above=above.unionPolygons(anti_overhang_[layer]);
						}
						else{
							above=above.unionPolygons(); //just to be sure the area is correctly unioned as otherwise difference may behave unexpectedly.
						}
						Polygons placeable =data[key].difference(above);
						data_placeable[RadiusLayerPair(radius,layer+1)]=data_placeable[RadiusLayerPair(radius,layer+1)].unionPolygons(placeable);
					}

				}

				for(size_t layer=min_layer_bottom;layer<=max_required_layer;layer++){
					key.second=layer;
					for(coord_t layer_offset=1;layer_offset<= z_distance_top_layers && layer_offset+layer <=max_required_layer;layer_offset++){
						data[key].add(data[RadiusLayerPair(radius,layer+layer_offset)]);
					}
					if(anti_overhang_.size()>layer)
					{
						data[key]=data[key].unionPolygons(anti_overhang_[layer].offset(radius));
					}
					else{
						data[key]=data[key].unionPolygons();
					}
				}

				for(LayerIndex layer=max_required_layer;layer>keys[i].second;layer--){
					data.erase(RadiusLayerPair(radius,layer)); //all these dont have the correct z_distance_top_layers as they can still have areas above them
				}

				for(auto pair:data){
					data_outer[pair.first]=data_outer[pair.first].unionPolygons(pair.second);
				}
				if(radius==0){
					for(auto pair:data_placeable){
						data_placeable_outer[pair.first]=data_placeable_outer[pair.first].unionPolygons(pair.second);
					}
				}
			}
#pragma omp critical(progress)
			{
				if(precalculated&&precalculation_progress<PROGRESS_PRECALC_COLL){

					precalculation_progress+=PROGRESS_PRECALC_COLL/keys.size();
					Progress::messageProgress(Progress::Stage::SUPPORT,precalculation_progress*progress_multiplier+progress_offset ,PROGRESS_TOTAL);
				}
			}
#pragma omp critical(collision_cache_)
{
	collision_cache_.insert(data_outer.begin(),data_outer.end());
}
		if(radius==0){

		#pragma omp critical(placeable_areas_cache_)
		{
				placeable_areas_cache_.insert(data_placeable_outer.begin(),data_placeable_outer.end());
		}
	}
	}
}


//ensures offsets are only done in sizes with a max step size per offset while adding the collision offset after each step, this ensures that areas cannot glitch through walls defined by the collision when offseting to fast
Polygons ModelVolumes::safeOffset(const Polygons& me,coord_t distance,ClipperLib::JoinType jt,coord_t max_safe_step_distance,const Polygons& collision)const{
	const size_t steps = std::abs (distance/max_safe_step_distance);
	assert(distance*max_safe_step_distance>=0);
	Polygons ret=me;

	for(size_t i=0;i<steps;i++){
		ret=ret.offset(max_safe_step_distance,jt).unionPolygons(collision);
	}
	ret= ret.offset(distance%max_safe_step_distance,jt);

	return ret.unionPolygons(collision);
}

void ModelVolumes::calculateAvoidance( std::deque<RadiusLayerPair> keys ){
#pragma omp for schedule(static,1) nowait
	for(long long unsigned int i=0;i<keys.size()*2;i++){ // calculating slow and fast avoidance in parallel to save increase performance if i is even is is normal if uneven slow
			coord_t radius = keys[i/2].first;
			LayerIndex max_required_layer=keys[i/2].second;
			bool slow=i%2;
			const coord_t offset_speed=slow?max_move_slow_:max_move_;
			const coord_t max_step_move = std::min(2*radius,current_min_xy_dist*2);
			RadiusLayerPair key(radius,0);
			Polygons last_layer;
			LayerIndex start_layer;
#pragma omp critical(avoidance_cache_slow_) //as i dont know of any way to change the name of a critical based on a variable i need to lock both, as this is the only order i lock is should never run into a deadlock
			{
#pragma omp critical(avoidance_cache_)
			{
				start_layer=1+getMaxCalculatedLayer(radius, slow?avoidance_cache_to_model_slow_:avoidance_cache_to_model_);
			}
			}
			if(start_layer>max_required_layer){
				logDebug("Requested calculation for value already calculated ?\n");
				continue;
			}
			start_layer=std::max(start_layer,LayerIndex(1));
			std::vector<std::pair<RadiusLayerPair,Polygons>> data(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));

			if(start_layer==1){
				last_layer=getCollision(radius, 0, true);
				last_layer.simplify(min_maximum_resolution_,min_maximum_deviation_);
				data[0]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);
			}
			else{
				last_layer=getAvoidance(radius, start_layer-1, slow , false, true); // minDist as the delta was already added
			}

			for(LayerIndex layer=start_layer;layer<=max_required_layer;layer++){
				key.second=layer;
				Polygons col=getCollision(radius, layer, true);
				last_layer=safeOffset(last_layer,-offset_speed,ClipperLib::jtRound,-max_step_move,col).smooth(5);
				last_layer.simplify(min_maximum_resolution_,min_maximum_deviation_);
				data[layer]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);

			}

#pragma omp critical(progress)
			{
				if(precalculated&&precalculation_progress<PROGRESS_PRECALC_COLL+PROGRESS_PRECALC_AVO){

					precalculation_progress+=support_rests_on_model?0.4:1 * PROGRESS_PRECALC_AVO/(keys.size()*2);
					Progress::messageProgress(Progress::Stage::SUPPORT,precalculation_progress*progress_multiplier+progress_offset ,PROGRESS_TOTAL);
				}
			}

	if(!slow){
#pragma omp critical(avoidance_cache_)
{
		avoidance_cache_.insert(data.begin(),data.end());
}
	}
	else{
#pragma omp critical(avoidance_cache_slow_)
		{
			avoidance_cache_slow_.insert(data.begin(),data.end());
		}

	}
	}

}

void ModelVolumes::calculatePlaceables( std::deque<RadiusLayerPair> keys ){
#pragma omp for schedule(static,1)
	for(long long unsigned int i=0;i<keys.size();i++){
			const coord_t radius = keys[i].first;
			const LayerIndex max_required_layer=keys[i].second;
			std::vector<std::pair<RadiusLayerPair,Polygons>> data(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			RadiusLayerPair key(radius,0);

			LayerIndex start_layer;
#pragma omp critical(placeable_areas_cache_)
			{
				start_layer=1+getMaxCalculatedLayer(radius, placeable_areas_cache_);
			}
			if(start_layer>max_required_layer){
				logDebug("Requested calculation for value already calculated ?\n");
				continue;
			}
			start_layer=std::max(start_layer,LayerIndex(1));

			for(LayerIndex layer=start_layer;layer<=max_required_layer;layer++){
				key.second=layer;
				Polygons placeable=getPlaceableAreas(0, layer);
				placeable.simplify(min_maximum_resolution_,min_maximum_deviation_); // it is faster to do this here in each thread than once in calculateCollision.
				placeable=placeable.offset(-radius).smooth(5);

				data[layer]=std::pair<RadiusLayerPair,Polygons>(key,placeable);
			}
#pragma omp critical(progress)
			{
				if(precalculated&&precalculation_progress<PROGRESS_PRECALC_COLL+PROGRESS_PRECALC_AVO){

					precalculation_progress+=0.2 * PROGRESS_PRECALC_AVO/(keys.size());
					Progress::messageProgress(Progress::Stage::SUPPORT,precalculation_progress*progress_multiplier+progress_offset ,PROGRESS_TOTAL);
				}
			}


#pragma omp critical(placeable_areas_cache_)
{
	placeable_areas_cache_.insert(data.begin(),data.end());
}

	}
}


void ModelVolumes::calculateAvoidanceToModel( std::deque<RadiusLayerPair> keys ){
#pragma omp for schedule(static,1)
	for(long long unsigned int i=0;i<keys.size()*2;i++){
			const coord_t radius = keys[i/2].first;
			const LayerIndex max_required_layer=keys[i/2].second;
			getPlaceableAreas(radius, max_required_layer);  // ensuring Placeableareas are calculated
			const bool slow=i%2;
			const coord_t offset_speed=slow?max_move_slow_:max_move_;
			const coord_t max_step_move = std::min(2*radius,current_min_xy_dist*2);
			Polygons last_layer;
			std::vector<std::pair<RadiusLayerPair,Polygons>> data(max_required_layer+1,std::pair<RadiusLayerPair,Polygons>(RadiusLayerPair(radius,-1),Polygons()));
			RadiusLayerPair key(radius,0);

			LayerIndex start_layer;
#pragma omp critical(avoidance_cache_to_model_slow_) //as i dont know of any way to change the name of a critical based on a variable i need to lock both, as this is the only order i lock is should never run into a deadlock
			{
#pragma omp critical(avoidance_cache_to_model_)
			{
				start_layer=1+getMaxCalculatedLayer(radius, slow?avoidance_cache_to_model_slow_:avoidance_cache_to_model_);
			}
			}
			if(start_layer>max_required_layer){
				logDebug("Requested calculation for value already calculated ?\n");
				continue;
			}
			start_layer=std::max(start_layer,LayerIndex(1));

			if(start_layer==1){
				last_layer=getCollision(radius, 0, true);
				last_layer.simplify(min_maximum_resolution_,min_maximum_deviation_);
				data[0]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);
			}
			else{
				last_layer=getAvoidance(radius, start_layer-1, slow , true, true); // minDist as the delta was already added
			}

			for(LayerIndex layer=start_layer;layer<=max_required_layer;layer++){
				key.second=layer;
				Polygons col=getCollision(radius, layer, true);
				last_layer=safeOffset(last_layer,-offset_speed,ClipperLib::jtRound,-max_step_move,col).difference(getPlaceableAreas(radius, layer)).smooth(5);

				last_layer.simplify(min_maximum_resolution_,min_maximum_deviation_);
				data[layer]=std::pair<RadiusLayerPair,Polygons>(key,last_layer);
			}
#pragma omp critical(progress)
			{
				if(precalculated&&precalculation_progress<PROGRESS_PRECALC_COLL+PROGRESS_PRECALC_AVO){

					precalculation_progress+=0.4 * PROGRESS_PRECALC_AVO/(keys.size()*2);
					Progress::messageProgress(Progress::Stage::SUPPORT,precalculation_progress*progress_multiplier+progress_offset ,PROGRESS_TOTAL);
				}
			}

	if(!slow){
#pragma omp critical(avoidance_cache_to_model_)
{
			avoidance_cache_to_model_.insert(data.begin(),data.end());
}
	}
	else{
#pragma omp critical(avoidance_cache_to_model_slow_)
{
			avoidance_cache_to_model_slow_.insert(data.begin(),data.end());
}
	}
	}
}


coord_t ModelVolumes::ceilRadius(coord_t radius) const {

	coord_t exponential_result=SUPPORT_TREE_EXPONENTIAL_THREASHOLD;
	if(radius>=SUPPORT_TREE_EXPONENTIAL_THREASHOLD&&SUPPORT_TREE_USE_EXPONENTIAL_COLLISION_RESOLUTION){
		while(exponential_result<radius||ignorable_radii_.count(exponential_result)){
			exponential_result=std::max(coord_t(exponential_result*SUPPORT_TREE_EXPONENTIAL_FACTOR),exponential_result+SUPPORT_TREE_COLLISION_RESOLUTION);
		}
		return exponential_result;
	}
	const auto remainder = radius % SUPPORT_TREE_COLLISION_RESOLUTION;
	const auto delta =
			remainder != 0 ? SUPPORT_TREE_COLLISION_RESOLUTION - remainder : 0;

	if(ignorable_radii_.count(radius + delta))
	{
		return ceilRadius(radius+delta+1);
	}

	return radius + delta;
}

const std::optional<std::reference_wrapper<const Polygons>> ModelVolumes::getArea(std::unordered_map<RadiusLayerPair, Polygons>& cache,const RadiusLayerPair key) const {
	const auto it = cache.find(key);
	if (it != cache.end()) {
		return std::optional<std::reference_wrapper<const Polygons>>{it->second};
	}
	else {
		return std::optional<std::reference_wrapper<const Polygons>>();
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
