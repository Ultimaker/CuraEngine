#include "polygonHelper.h"

namespace cura {

PolygonHelper::PolygonHelper()
{

}

void PolygonHelper::savePartsToFile(SliceDataStorage& storage)
{
    Point3 modelMin = storage.modelMin;
    Point3 modelSize = storage.modelSize;
    ofstream partFile;

    string filePath = CURA_DEBUG_ROOT_OUTPUT_FILE_PATH + "/" + CURA_DEBUG_FILE_NAME_PARTS;
    // open file
    partFile.open (filePath.c_str());
    // output model size
    partFile << "model size:" << modelSize.x << " " << modelSize.y <<endl;
    // loop through volumes
    for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
    {
        // output current volumen index
        partFile << "volume index:" << volumeIdx <<endl;
        // loop through layers
        for(unsigned int layerNr=0; layerNr<storage.volumes[volumeIdx].layers.size(); layerNr++)
        {
            // output current index of layer
            partFile << "layer index:" << layerNr <<endl;
            SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
            // loop through parts
            for(unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
            {
                // output current index of part
                partFile << "part index:" << partNr <<endl;
                SliceLayerPart* part = &layer->parts[partNr];
                for(unsigned int polygonNr = 0; polygonNr < part->outline.size(); polygonNr++)
                {
                    // output current outline
                    partFile << "outline index:" << polygonNr <<endl;
                    // the first polygon is the outer wall of the part!
                    for(unsigned int pointNr = 0; pointNr < part->outline[polygonNr].size(); pointNr++)
                    {
                        partFile << (part->outline[polygonNr][pointNr].X - modelMin.x)
                                 << " "
                                 << (part->outline[polygonNr][pointNr].Y - modelMin.y)
                                 <<" ";
                    }
                    // end of outline
                    partFile<<endl;
                }
            }
        }

    }
    // close file
    partFile.close();
}

}
