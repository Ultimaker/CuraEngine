#ifndef SLICE_DATA_PROCESSOR_H
#define SLICE_DATA_PROCESSOR_H

#include <string>

namespace cura
{

class SettingsBaseVirtual;
class SliceDataStorage;
class MeshGroup;
class Settings;

class SliceDataProcessor
{
public:
    static bool loadSlices(MeshGroup& meshgroup, const std::string& filename, Settings& object_parent_settings, bool support_slices = false);
    static bool exportSlices(const SliceDataStorage& storage, const std::string& filename, bool support_slices = false);
};

}//namespace cura

#endif//SLICE_DATA_PROCESSOR_H
