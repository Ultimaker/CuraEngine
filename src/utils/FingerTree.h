/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_FINGER_TREE_H
#define UTILS_FINGER_TREE_H

#include <vector>
#include <cassert>

namespace cura
{

/*!
 * A tree with nodes with a constant number of children.
 * 
 * Efficient storage and lookup times.
 * Works best for balanced trees.
 * Constructs nodes automatically using a default value.
 * 
 * Example usage:
 * 
 *  int uninitialized = -1;
 *  FingerTree<int> tree(2, 3, uninitialized);
 *  for (Node grand_child_it = tree.begin(2); grand_child_it != tree.end(2); ++grand_child_it)
 *      if (*grand_child_it == uninitialized)
 *          *grand_child_it = 123;
 * 
 */
template<typename T>
class FingerTree
{
public:
    class Node; // forward decl
    friend class FingerTree<T>::Node;

    FingerTree(unsigned int finger_count, size_t initial_capacity = 1, const T& default_value = T());

    Node getRoot();
    void setRoot(T&& root_node);

    void debugOutput();

    /*!
     * Get begin iterator over all nodes of a specific depth
     * 
     * usage:
     * 
     * for (Node grand_child_it = tree.begin(2); grand_child_it != tree.end(2); ++grand_child_it)
     *     *grand_child_it = something();
     */
    Node begin(unsigned int depth);
    /*!
     * Get end iterator over all nodes of a specific depth
     * 
     * usage:
     * 
     * for (Node grand_child_it = tree.begin(2); grand_child_it != tree.end(2); ++grand_child_it)
     *     *grand_child_it = something();
     */
    Node end(unsigned int depth);
    
protected:
    unsigned int finger_count; //!< The amount of fingers on each node
    std::vector<T> elements; //!< The elements stored in a breadth first search manner (if the tree would be full)
    const T default_value; //!< The default value to use when expanding the capacity of the tree.
};

/*!
 * horizontal iterator over a finger tree.
 * 
 * usage:
 * 
 * Node node = getSpecificNode();
 * for (T& child : node)
 *     child = getSomeValue();
 */
template<typename T>
class FingerTree<T>::Node
{
public:
    Node(FingerTree<T>* tree, size_t vector_index)
    : tree(tree)
    , vector_index(vector_index)
    {}
    T& operator *() const
    {
        if (vector_index >= tree->elements.size())
        {
            tree->elements.resize(vector_index + 1, tree->default_value);
        }
        assert(vector_index < tree->elements.size());
        return tree->elements[vector_index];
    }
    T* operator ->() const
    {
        return &this->operator*();
    }
    T& get()
    {
        return tree->elements[vector_index];
    }
    Node& operator=(T&& elem)
    {
        if (vector_index >= tree->elements.size())
        {
            tree->elements.resize(vector_index + 1);
        }
        assert(vector_index < tree->elements.size());
        tree->elements[vector_index] = std::move(elem);
        return *this;
    }
    Node operator[](unsigned int child_idx)
    {
        size_t vector_index = getChildIndex(child_idx);
        if (vector_index >= tree->elements.size())
        {
            tree->elements.resize(vector_index + 1);
        }
        assert(vector_index < tree->elements.size());
        return Node(tree, vector_index);
    }
    
    Node& operator++()
    {
        vector_index++;
        return *this;
    }
    Node operator++(int)
    {
        Node tmp(*this);
        operator++();
        return tmp;
    }
    Node& operator--()
    {
        vector_index--;
        return *this;
    }
    Node operator--(int)
    {
        Node tmp(*this);
        operator--();
        return tmp;
    }
    
    Node begin()
    {
        return Node(tree, getChildIndex(0));
    }
    Node end()
    {
        return Node(tree, getChildIndex(tree->finger_count - 1) + 1);
    }
    
    bool operator==(const Node& rhs)
    {
        return vector_index == rhs.vector_index && tree == rhs.tree;
    }
    bool operator!=(const Node& rhs)
    {
        return !(*this == rhs);
    }
    
    size_t getIndex()
    {
        return vector_index;
    }
protected:
    size_t getChildIndex(unsigned int child_idx)
    {
        assert(child_idx < tree->finger_count);
        return vector_index * tree->finger_count + child_idx + 1;
    }
    FingerTree<T>* tree; //!< pointer to the tree
    size_t vector_index; //!< index in the vector of all elements: \ref FingerTree::elements
    
};


template<typename T>
FingerTree<T>::FingerTree(unsigned int finger_count, size_t initial_capacity, const T& default_value)
: finger_count(finger_count)
, elements(initial_capacity, default_value)
, default_value(default_value)
{
}

template<typename T>
typename FingerTree<T>::Node FingerTree<T>::getRoot()
{
    return Node(this, 0);
}

template<typename T>
void FingerTree<T>::setRoot(T&& root_node)
{
    assert(elements.size() > 0);
    elements[0] = std::move(root_node);
}


template<typename T>
typename FingerTree<T>::Node FingerTree<T>::begin(unsigned int depth)
{
    unsigned int elems_per_level = 1;
    unsigned int level_start_index = 0;
    for (unsigned int d = 0; d < depth; d++)
    {
        level_start_index += elems_per_level;
        elems_per_level *= finger_count;
    }
    return Node(this, level_start_index);
}

template<typename T>
typename FingerTree<T>::Node FingerTree<T>::end(unsigned int depth)
{
    unsigned int elems_per_level = 1;
    unsigned int level_start_index = 0;
    for (unsigned int d = 0; d < depth; d++)
    {
        level_start_index += elems_per_level;
        elems_per_level *= finger_count;
    }
    return Node(this, level_start_index + elems_per_level);
}

template<typename T>
void FingerTree<T>::debugOutput()
{
    unsigned int next_depth = 1;
    unsigned int next_depth_idx = 1;
    for (unsigned int elem_idx = 0; elem_idx < elements.size(); elem_idx++)
    {
        if (elem_idx == next_depth_idx)
        {
            std::cout << '\n';
            next_depth *= finger_count;
            next_depth_idx += next_depth;
        }
        T& elem = elements[elem_idx];
        std::cout << elem << " ";
    }
}

}//namespace cura
#endif//UTILS_FINGER_TREE_H

