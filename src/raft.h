/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef RAFT_H
#define RAFT_H

#include "sliceDataStorage.h"

namespace cura {

void generateRaft(SliceDataStorage& storage, int distance, int extrusionWidth);

}//namespace cura

#endif//RAFT_H
