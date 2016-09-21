#ifndef PRIME_TOWER_H
#define PRIME_TOWER_H

#include "GCodePathConfig.h"
#include "MeshGroup.h"
#include "utils/polygon.h" // Polygons

namespace cura 
{

    
class SliceDataStorage;
class GCodePlanner;
class GCodeExport;

typedef std::vector<IntPoint> PolyLine;

class PrimeTower
{
private:
    int extruder_count;
    std::vector<GCodePathConfig> config_per_extruder;

    Point wipe_point;

    class WallInfill
    {
        
    };
public:
    void initConfigs(MeshGroup* meshgroup, std::vector<RetractionConfig>& retraction_config_per_extruder);
    void setConfigs(MeshGroup* configs, int layer_thickness);
    
    Polygons ground_poly;
    
    std::vector<PolyLine> extruder_paths;
    
    
    void generateGroundpoly(SliceDataStorage& storage);

    std::vector<std::vector<Polygons>> patterns_per_extruder; //!< for each extruder a vector of patterns to alternate between, over the layers
    
    void generatePaths3(SliceDataStorage& storage);
    
    void generatePaths2(SliceDataStorage& storage);
    /*!
     * Generate the area where the prime tower should be.
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param total_layers The total number of layers 
     */
    void generatePaths(SliceDataStorage& storage, unsigned int total_layers);
    void generatePaths_OLD(SliceDataStorage& storage, unsigned int total_layers);

    void computePrimeTowerMax(SliceDataStorage& storage);
    
    PrimeTower();

    void addToGcode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed);
    void addToGcode_OLD(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed);
    void addToGcode3(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed);

};




}//namespace cura

#endif // PRIME_TOWER_H