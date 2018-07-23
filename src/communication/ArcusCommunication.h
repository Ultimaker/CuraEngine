//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ARCUSCOMMUNICATION_H
#define ARCUSCOMMUNICATION_H

#include <queue> //To queue up slice commands.

#include "Communication.h" //The class we're implementing.

namespace cura
{
//Forward declarations to speed up compilation.
class Slice;

/*
 * Communication class that connects via libArcus to Cura's front-end.
 */
class ArcusCommunication : public Communication
{
public:
    /*
     * \brief Test if there are any more slices in the queue.
     */
    const bool hasSlice() const;

private:
    /*
     * A queue of slices that are still left to do.
     */
    std::queue<Slice> to_slice;
};

} //namespace cura

#endif //ARCUSCOMMUNICATION_H