#include "SliceDataProcessor.h"
#include "utils/serialization.h"
#include "sliceDataStorage.h"

#include <fstream>

namespace cura
{

bool SliceDataProcessor::exportSlices(const SliceDataStorage& storage, const std::string& filename, bool support_slices /*= false*/)
{
    geometry::proto::Slices slices;
    if (!support_slices)
    {
        slices = serialization_utils::getSlices(storage);
    }
    else
    {
        slices = serialization_utils::getSupportSlices(storage);
    }

    std::ofstream stream(filename, std::ios_base::binary | std::ios_base::out);
    if (!slices.SerializeToOstream(&stream))
    {
        return false;
    }

    return true;
}

}//namespace cura
