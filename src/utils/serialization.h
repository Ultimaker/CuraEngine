#ifndef SERIALIZATION_UTILS_H
#define SERIALIZATION_UTILS_H

#include "SliceData.pb.h"

namespace cura
{

class SliceDataStorage;

namespace serialization_utils
{

geometry::proto::Slices getSlices(const SliceDataStorage& storage);
geometry::proto::Slices getSupportSlices(const SliceDataStorage& storage);

}//namespace serialization_utils

}//namespace cura

#endif //SERIALIZATION_UTILS_H
