//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "layerPart.h"
#include "sliceDataStorage.h"
#include "slicer.h"
#include "settings/EnumSettings.h" //For ESurfaceMode.
#include "settings/Settings.h"
#include "progress/Progress.h"
#include "utils/SVG.h" // debug output
#include <memory> // for unique_ptr
#include <sys/stat.h> // for mkdir
#include <iostream> // for cout
#include "utils/logoutput.h" // for log
#include <fstream> // for ofstream
using namespace std;

/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons, and makes groups of polygons,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundary of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/

namespace cura {

void createLayerWithParts(const Settings& settings, int layer_nr, SliceLayer& storageLayer, SlicerLayer* layer)
{
    cout << "createLayerWithParts for " << layer_nr << endl;

    storageLayer.openPolyLines = layer->openPolylines;

    const bool union_all_remove_holes = settings.get<bool>("meshfix_union_all_remove_holes");
    cout << "union_all_remove_holes: " << union_all_remove_holes << endl;
    if (union_all_remove_holes)
    {
        for(unsigned int i=0; i<layer->polygons.size(); i++)
        {
            if (layer->polygons[i].orientation())
                layer->polygons[i].reverse();
        }
    }

    std::vector<PolygonsPart> result;
    const bool union_layers = settings.get<bool>("meshfix_union_all");
    cout << "union_layers: " << union_layers << endl;
    const ESurfaceMode surface_only = settings.get<ESurfaceMode>("magic_mesh_surface_mode");
    
    switch (surface_only)
    {
    case ESurfaceMode::SURFACE:
        cout << "surface_only: SURFACE" << endl;
        break;
    case ESurfaceMode::NORMAL:
        cout << "surface_only: NORMAL" << endl;
        break;
    case ESurfaceMode::BOTH:
        cout << "surface_only: BOTH" << endl;
        break;    
    default:
        cout << "surface_only: UNKNOWN" << endl;
    }

    if (surface_only == ESurfaceMode::SURFACE && !union_layers)
    { // Don't do anything with overlapping areas; no union nor xor
        cout << "don't do anything with overlapping areas" << endl;
        result.reserve(layer->polygons.size());
        for (const PolygonRef poly : layer->polygons)
        {
            result.emplace_back();
            result.back().add(poly);
        }
    }
    else
    {
        cout << "splitIntoParts(" << union_layers << " || " << union_all_remove_holes << ")" << endl;
        result = layer->polygons.splitIntoParts(union_layers || union_all_remove_holes);
    }
    cout << "result size is " << result.size() << endl;
    const coord_t hole_offset = settings.get<coord_t>("hole_xy_offset");
    cout << "hole_offset: " << hole_offset << endl;
    for(unsigned int i=0; i<result.size(); i++)
    {
        storageLayer.parts.emplace_back();
        if (hole_offset != 0)
        {
            // holes are to be expanded or shrunk
            PolygonsPart outline;
            Polygons holes;
            for (const PolygonRef poly : result[i])
            {
                if (poly.orientation())
                {
                    outline.add(poly);
                }
                else
                {
                    holes.add(poly.offset(hole_offset));
                }
            }
            for (PolygonRef hole : holes.unionPolygons().intersection(outline))
            {
                hole.reverse();
                outline.add(hole);
            }
            storageLayer.parts[i].outline = outline;
        }
        else
        {
            storageLayer.parts[i].outline = result[i];
        }
        storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
    }
}
void createLayerParts(SliceMeshStorage& mesh, Slicer* slicer)
{
    const auto total_layers = slicer->layers.size();
    assert(mesh.layers.size() == total_layers);

    // OpenMP compatibility fix for GCC <= 8 and GCC >= 9
    // See https://www.gnu.org/software/gcc/gcc-9/porting_to.html, section "OpenMP data sharing"
#if defined(__GNUC__) && __GNUC__ <= 8
    #pragma omp parallel for default(none) shared(mesh, slicer) schedule(dynamic)
#else
    #pragma omp parallel for default(none) shared(mesh, slicer, total_layers) schedule(dynamic)
#endif // defined(__GNUC__) && __GNUC__ <= 8
    // Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
    for (int layer_nr = 0; layer_nr < static_cast<int>(total_layers); layer_nr++)
    {
        SliceLayer& layer_storage = mesh.layers[layer_nr];
        SlicerLayer& slice_layer = slicer->layers[layer_nr];
        createLayerWithParts(mesh.settings, layer_nr, layer_storage, &slice_layer);
    }

    for (LayerIndex layer_nr = total_layers - 1; layer_nr >= 0; layer_nr--)
    {
        SliceLayer& layer_storage = mesh.layers[layer_nr];
        if (layer_storage.parts.size() > 0 || (mesh.settings.get<ESurfaceMode>("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer_storage.openPolyLines.size() > 0))
        {
            mesh.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
            break;
        }
    }
}

int nextColorIdx = 0;
const int numColors = 7;
const SVG::Color colors[numColors] = {
    SVG::Color::GRAY,
    SVG::Color::RED,
    SVG::Color::BLUE,
    SVG::Color::GREEN,
    SVG::Color::ORANGE,
    SVG::Color::MAGENTA,
    SVG::Color::YELLOW,
};
    
void layerparts2HTML(SliceDataStorage& storage, const char* dir)
{
    Point3 modelSize = storage.model_size;
    Point3 modelMin = storage.model_min;
    Point model_min_2d = Point(modelMin.x, modelMin.y);
    Point model_max_2d = Point(modelSize.x, modelSize.y) + model_min_2d;
    AABB aabb(model_min_2d, model_max_2d);
    char path [50];
    
    mkdir(dir, 0777);
    sprintf(path, "%s/svgs", dir);
    mkdir(path, 0777);

    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        for(unsigned int layer_idx = 0; layer_idx < mesh.layers.size(); layer_idx++)
        {
            SliceLayer& layer = mesh.layers[layer_idx];
            sprintf(path, "%s/svgs/%04d-%04d.svg", dir, mesh_idx, layer_idx);
            unique_ptr<SVG> svg(new SVG(path, aabb));
            nextColorIdx=0;
            
            for(unsigned int part_idx = layer.parts.size(); part_idx > 0; part_idx--)
            {
                SliceLayerPart& part = layer.parts[part_idx-1];
                // for (unsigned int skin_idx = 0; skin_idx < part.skin_parts.size(); skin_idx++)
                // {
                //     SkinPart& skin = part.skin_parts[skin_idx];
                //     cout << "skin part index " << skin_idx << endl;
                // }

                cout << "HELLO!!!!!!!" << endl;

                svg->writeComment("part.outline");
                svg->writeAreas(part.outline, colors[nextColorIdx++]);                
                svg->writeComment("part.infill_area");
                svg->writeAreas(part.infill_area, colors[nextColorIdx++]);
                svg->writeComment("part.print_outline");
                svg->writeAreas(part.print_outline, colors[nextColorIdx++]);
                svg->writeComment("part.getOwnInfillArea");
                svg->writeAreas(part.getOwnInfillArea(), colors[nextColorIdx++]);                
                for (unsigned int i = 0; i < part.spaghetti_infill_volumes.size(); i++)
                {
                    svg->writeComment("part.spaghetti_infill_volumes");
                    svg->writeAreas(part.spaghetti_infill_volumes[i].first, colors[nextColorIdx++]);
                }
            }
        }
    }

    sprintf(path, "%s/index.html", dir);
    ofstream f(path);

    f << "<!DOCTYPE html>" << endl;
    f << "<html>" << endl;
    f << "<head>" << endl;
    f << "    <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>" << endl;
    f << "    <script>" << endl;
    f << "        svgs = [" << endl;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        for(unsigned int layer_idx = 0; layer_idx < mesh.layers.size(); layer_idx++)
        {
            sprintf(path, "svgs/%04d-%04d.svg", mesh_idx, layer_idx);
            f << "'" << path << "'," << endl;
        }
    }
    f << "        ];" << endl;
    f << "        $(document).ready(function(){" << endl;
    f << "            $('#layer').on('input', function(){" << endl;
    f << "                $('#span').text(this.value);" << endl;
    f << "                $('#img').attr({'src': svgs[this.value-1]});" << endl;
    f << "            });" << endl;
    f << "            $('#layer').attr({'max': svgs.length, 'min': 1, 'value': 0}).trigger('input');" << endl;
    f << "        });" << endl;
    f << "    </script>" << endl;
    f << "</head>" << endl;
    f << "<body>" << endl;
    f << "<h3>Layer <span id='span'>0</span></h3>" << endl;
    f << "<input type='range' id='layer' name='layer' min='0' max='50'>" << endl;
    f << "<hr><img id='img'></html>" << endl;
}

}//namespace cura
