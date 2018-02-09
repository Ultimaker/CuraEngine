//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "UnionFind.h" //The header we're implementing.

namespace cura
{

template <class E>
size_t UnionFind<E>::add(const E& item)
{
    items.push_back(item);
    size_t set_handle = parent_index.size(); //Guaranteed to be unique because there has never been any item with this index (can't remove from this data structure!)
    parent_index.push_back(set_handle);
    return set_handle;
}

template <class E>
size_t UnionFind<E>::find(const E& item) const
{
    const size_t index = element_to_position[item];
    const size_t parent = parent_index[index];
    if (parent == (size_t)-1) //This is a root.
    {
        return index;
    }
    //TODO: Implement path compression.
    return find(parent);
}

template <class E>
size_t UnionFind<E>::unite(const size_t first, const size_t second)
{
    parent_index[first] = second;
    return second;
}

}