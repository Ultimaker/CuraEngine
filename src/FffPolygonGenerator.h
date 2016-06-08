#ifndef FFF_AREA_GENERATOR_H
#define FFF_AREA_GENERATOR_H


#include "MeshGroup.h"
#include "utils/polygonUtils.h"
#include "utils/NoCopy.h"
#include "utils/gettime.h"
#include "settings/settings.h"
#include "sliceDataStorage.h"
#include "commandSocket.h"
#include "PrintFeature.h"
#include "progress/ProgressEstimator.h"
#include "progress/ProgressStageEstimator.h"

namespace cura
{

/*!
 * Primary stage in Fused Filament Fabrication processing: Polygons are generated.
 * The model is sliced and each slice consists of polygons representing the outlines: the boundaries between inside and outside the object.
 * After slicing, the layers are processed; for example the wall insets are generated, and the areas which are to be filled with support and infill, which are all represented by polygons.
 * In this stage nothing other than areas and circular paths are generated, which are both represented by polygons.
 * No infill lines or support pattern etc. is generated.
 * 
 * The main function of this class is FffPolygonGenerator::generateAreas().
 */
class FffPolygonGenerator : public SettingsMessenger, NoCopy
{
public:
    /*!
     * Basic constructor
     */
    FffPolygonGenerator(SettingsBase* settings_)
    : SettingsMessenger(settings_)
    {
    }

    /*!
     * Slice the \p object, process the outline information into inset perimeter polygons, support area polygons, etc. 
     * 
     * \param object The object to slice.
     * \param timeKeeper Object which keeps track of timings of each stage.
     * \param storage Output parameter: where the outlines are stored. See SliceLayerPart::outline.
     */
    bool generateAreas(SliceDataStorage& storage, MeshGroup* object, TimeKeeper& timeKeeper);
  
private:
    
    /*!
     * Slice the \p object and store the outlines in the \p storage.
     * 
     * \param object The object to slice.
     * \param timeKeeper Object which keeps track of timings of each stage.
     * \param storage Output parameter: where the outlines are stored. See SliceLayerPart::outline.
     * 
     * \return Whether the process succeeded (always true).
     */
    bool sliceModel(MeshGroup* object, TimeKeeper& timeKeeper, SliceDataStorage& storage); /// slices the model

    /*!
     * Processes the outline information as stored in the \p storage: generates inset perimeter polygons, support area polygons, etc. 
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param timeKeeper Object which keeps track of timings of each stage.
     */
    void slices2polygons(SliceDataStorage& storage, TimeKeeper& timeKeeper);
    
    /*!
     * Processes the outline information as stored in the \p storage: generates inset perimeter polygons, skin and infill
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param mesh_idx The index of the mesh to process in the vector of meshes in \p storage
     * \param mesh_order The order in which the meshes are processed (used for infill meshes)
     * \param total_layers The total number of layers over all objects
     * \param inset_skin_progress_estimate The progress stage estimate calculator
     */
    void processBasicWallsSkinInfill(SliceDataStorage& storage, unsigned int mesh_idx, std::vector<unsigned int>& mesh_order, size_t total_layers, ProgressStageEstimator& inset_skin_progress_estimate);
    
    /*!
     * Process the mesh to be an infill mesh: limit all outlines to within the infill of normal meshes and subtract their volume from the infill of those meshes
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param mesh_idx The index of the mesh to process in the vector of meshes in \p storage
     * \param mesh_order The order in which the meshes are processed
     * \param total_layers The total number of layers over all objects
     */
    void processInfillMesh(SliceDataStorage& storage, unsigned int mesh_idx, std::vector<unsigned int>& mesh_order, size_t total_layers);
    
    /*!
     * Process features which are derived from the basic walls, skin, and infill:
     * fuzzy skin, infill combine
     * 
     * \param mesh Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param total_layers The total number of layers over all objects
     */
    void processDerivedWallsSkinInfill(SliceMeshStorage& mesh, size_t total_layers);
    
    /*!
     * Remove all bottom layers which are empty.
     * 
     * \warning Changes \p total_layers
     * 
     * \param storage Input and Ouput parameter: stores all layers
     * \param layer_height The height of each layer
     * \param total_layers The total number of layers
     */
    void removeEmptyFirstLayers(SliceDataStorage& storage, const int layer_height, unsigned int& total_layers);
    
    /*!
     * Generate the inset polygons which form the walls.
     * \param mesh Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param layer_nr The layer for which to generate the insets.
     */
    void processInsets(SliceMeshStorage& mesh, unsigned int layer_nr);

    /*!
     * Generate the outline of the ooze shield.
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param total_layers The total number of layers 
     */
    void processOozeShield(SliceDataStorage& storage, unsigned int total_layers);
    
    /*!
     * Generate the skin areas.
     * \param mesh Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param layer_nr The layer for which to generate the skin areas.
     */
    void processSkinsAndInfill(SliceMeshStorage& mesh, unsigned int layer_nr); 

    /*!
     * Generate the polygons where the draft screen should be.
     * 
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     * \param total_layers The total number of layers 
     */
    void processDraftShield(SliceDataStorage& storage, unsigned int total_layers);
    /*!
     * Generate the skirt/brim/raft areas/insets.
     * \param storage Input and Output parameter: fetches the outline information (see SliceLayerPart::outline) and generates the other reachable field of the \p storage
     */
    void processPlatformAdhesion(SliceDataStorage& storage);
    
    
    
    /*!
     * Make the outer wall 'fuzzy'
     */
    void processFuzzyWalls(SliceMeshStorage& mesh);
    
    

    
};
}//namespace cura
#endif // FFF_AREA_GENERATOR_H
