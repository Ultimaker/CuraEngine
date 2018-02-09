//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UNIONFIND_H
#define UNIONFIND_H

#include <stddef.h> //For size_t.
#include <vector> //Holds the main data.
#include <unordered_map> //To map the data type to indices for user's convenience.

namespace cura
{

/*!
 * A union-find data structure.
 *
 * This data structure keeps track of a set of sets. The sets can be combined in
 * constant time. Which set an item is part of can be found in amortised
 * constant time.
 *
 * This data structure is not thread-safe.
 */
template <typename E>
class UnionFind
{
public:
    /*!
     * Adds a new item to the union-find data structure.
     *
     * The item will be placed in a singleton set until it is united with
     * another set.
     * \param item The item to add.
     * \return The handle of the set that the item gets placed in. This can be
     * used to combine its set with another set.
     */
    size_t add(const E item)
    {
        items.push_back(item);
        size_t handle = parent_index.size(); //Guaranteed to be unique because there has never been any item with this index (can't remove from this data structure!)
        element_to_position[item] = handle;
        std::cout << "Adding item " << (size_t)&item << std::endl;
        parent_index.push_back(handle);
        return handle;
    }

    /*!
     * Finds the set that an item is part of.
     * \param item The item to find the set of.
     * \return The handle of the set that the item is part of. Compare this to
     * the handles of the sets that other items are part of to determine if they
     * are in the same set.
     */
    size_t find(const E& item) const
    {
        std::cout << "Finding item " << (size_t)&item << std::endl;
        const typename std::unordered_map<E, size_t>::const_iterator it = element_to_position.find(item);
        if (it == element_to_position.end())
        {
            return -1;
        }
        const size_t index = it->second;
        return find(index);
    }

    /*!
     * Finds the set that an item is part of.
     * \param item_handle The handle of an item, as returned by the add method.
     * \return The handle of the set that the item is part of. Compare this to
     * the handles of the sets that other items are part of to determine if they
     * are in the same set.
     */
    size_t find(const size_t item_handle) const
    {
        const size_t parent = parent_index[item_handle];
        if (parent == (size_t)-1) //This is a root.
        {
            return item_handle;
        }
        //TODO: Implement path compression.
        return find(parent);
    }

    /*!
     * Unite two sets to be together in one set.
     * \param first One of the sets to combine with the other.
     * \param second The other set to combine with the first.
     * \return The new handle for the combined set.
     */
    size_t unite(const size_t first, const size_t second)
    {
        parent_index[first] = second;
        return second;
    }

private:
    /*!
     * Holds all items in the entire data structure.
     */
    std::vector<E> items;

    /*!
     * Tracks where each element is, so that we can find it back when the user
     * only specifies an element parameter.
     */
    std::unordered_map<E, size_t> element_to_position;

    /*!
     * For each item, the set handle of the parent item.
     *
     * Items belong to the set of their most ancient ancestor, so to find if two
     * items are in the same set, find if they have any ancestor in common.
     * Items with parent -1 (or actually max of size_t since it's unsigned) are
     * root. There may be multiple roots.
     */
    std::vector<size_t> parent_index;
};

}

#endif /* UNIONFIND_H */