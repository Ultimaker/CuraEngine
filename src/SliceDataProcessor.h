#ifndef SLICE_DATA_PROCESSOR_H
#define SLICE_DATA_PROCESSOR_H

#include <string>

namespace cura
{

class SettingsBaseVirtual;
class SliceDataStorage;

class SliceDataProcessor
{
public:
    static bool exportSlices(const SliceDataStorage& storage, const std::string& filename, bool support_slices = false);
};

}//namespace cura

#endif//SLICE_DATA_PROCESSOR_H
