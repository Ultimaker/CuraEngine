//Copyright (c) 2018 Ultimaker
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_FINGER_TREE_H
#define UTILS_FINGER_TREE_H

#include <array>

namespace cura
{

template<typename Elem, unsigned int finger_count>
class FingerTree
{
public:

    struct Node
    {
        Elem elem;
        std::array<Node*, finger_count> children;

        Node(Elem elem)
        : elem(elem)
        {
            children.fill(nullptr);
        }
    };
    
    Node* root;
    
    FingerTree()
    : root(nullptr)
    {
    }
    
private:
    
};

}//namespace cura

#endif//UTILS_FINGER_TREE_H
