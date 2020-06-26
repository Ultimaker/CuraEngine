#include "SliceDataProcessor.h"
#include "utils/serialization.h"
#include "sliceDataStorage.h"
#include "MeshGroup.h"
#include "settings/Settings.h"

#include <algorithm>
#include <iterator>
#include <fstream>

namespace cura
{

bool SliceDataProcessor::loadSlices(MeshGroup& meshgroup, const std::string& filename, Settings& object_parent_settings, bool support_slices/* = false*/)
{
    std::ifstream stream(filename, std::ios_base::in | std::ios_base::binary);
    if (!stream.is_open())
    {
        return false;
    }

    std::vector<Mesh> meshes;
    if (!serialization_utils::parseSlicesFromStream(stream, meshgroup, meshes, object_parent_settings, support_slices))
    {
        return false;
    }

    if (!support_slices && meshes.empty())
    {
        return false;
    }

    std::copy(std::make_move_iterator(meshes.begin()),
              std::make_move_iterator(meshes.end()),
              std::back_inserter(meshgroup.meshes));

    size_t largest_layer_count = std::max_element(meshgroup.meshes.begin(), meshgroup.meshes.end(), [](const Mesh& lhs, const Mesh& rhs){
        return lhs.layers.size() < rhs.layers.size();
    })->layers.size();

    for (auto& mesh : meshgroup.meshes)
    {
        if (mesh.layers.size() < largest_layer_count)
            mesh.layers.resize(largest_layer_count);
    }

    if (support_slices)
    {
        for (auto& mesh : meshgroup.meshes)
            mesh.predefined_support_slices = true;
    }

    return true;
}

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
