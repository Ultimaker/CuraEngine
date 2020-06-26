#ifndef SERIALIZATION_UTILS_H
#define SERIALIZATION_UTILS_H

#include <istream>
#include "../mesh.h"
#include "SliceData.pb.h"

namespace cura
{

class SliceDataStorage;
class MeshGroup;
class Settings;

namespace serialization_utils
{

bool parseSlicesFromStream(std::istream& stream, MeshGroup& meshgroup, std::vector<Mesh>& meshes, Settings& object_parent_settings, bool support_slices = false);
geometry::proto::Slices getSlices(const SliceDataStorage& storage);
geometry::proto::Slices getSupportSlices(const SliceDataStorage& storage);

}//namespace serialization_utils

}//namespace cura

#endif //SERIALIZATION_UTILS_H
