#ifndef POLYGONHELPER_H
#define POLYGONHELPER_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

#include "utils/polygon.h"
#include "sliceDataStorage.h"
/**
 * @brief The PolygonHelper class helper method for polygon
 */
namespace cura {

const string CURA_DEBUG_FILE_NAME_PARTS = "parts_by_layers.txt";
const string CURA_DEBUG_ROOT_OUTPUT_FILE_PATH = "/tmp";

class PolygonHelper
{
public:
    PolygonHelper();
    /**
     * @brief savePartsToFile save the closed polygons of parts into external text file. See file format in https://gist.github.com/rickyzhang82/33831c6d5a3eaaa3de3ffb5122f15b69
     * @param storage
     */
    static void savePartsToFile(SliceDataStorage& storage);
};

}
#endif // POLYGONHELPER_H
