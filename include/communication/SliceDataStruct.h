//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SLICEDATASTRUCT_H
#define SLICEDATASTRUCT_H
#ifdef ARCUS

#include <memory> //To store pointers to slice data.
#include <unordered_map> //To store the slice data by layer.

namespace cura
{

/*!
 * \brief A template structure used to store data to be sent to the front end.
 */
template <typename T>
class SliceDataStruct
{
    //You are not allowed to copy this data, so no copy constructor or assignment operator.
    SliceDataStruct(const SliceDataStruct&) = delete;
    SliceDataStruct& operator=(const SliceDataStruct&) = delete;
public:
    /*
     * \brief Creates a new, empty structure with slice data.
     */
    SliceDataStruct()
        : sliced_objects(0)
        , current_layer_count(0)
        , current_layer_offset(0)
    { }

    size_t sliced_objects; //!< The number of sliced objects for this sliced object list.

    size_t current_layer_count; //!< Number of layers for which data has been buffered in slice_data so far.
    size_t current_layer_offset; //!< Offset to add to layer number for the current slice object when slicing one at a time.

    /*
     * The slice data itself, which can be of any type.
     */
    std::unordered_map<int, std::shared_ptr<T>> slice_data;
};

}

#endif //ARCUS
#endif //SLICEDATASTRUCT_H