#ifndef PRIME_TOWER_H
#define PRIME_TOWER_H

#include "gcodeExport.h" // GCodePathConfig
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
    unsigned int extruder_count;
    std::vector<GCodePathConfig> config_per_extruder;

public:
    void setConfigs(MeshGroup* configs, RetractionConfig* retraction_config, int layer_thickness);
    
    Polygons ground_poly;
    
    std::vector<PolyLine> extruder_paths;
    
    void generatePaths2(SliceDataStorage& storage);
    /*!
     * Generate the area where the prime tower should be.
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param totalLayers The total number of layers 
     */
    void generatePaths(SliceDataStorage& storage, unsigned int totalLayers);

    void computePrimeTowerMax(SliceDataStorage& storage);
    
    PrimeTower(MeshGroup* configs, RetractionConfig* retraction_config);

    void addToGcode(SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed);

};




}//namespace cura

#endif // PRIME_TOWER_H