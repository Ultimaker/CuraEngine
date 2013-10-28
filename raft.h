/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef RAFT_H
#define RAFT_H

#include "sliceDataStorage.h"

void generateRaft(SliceDataStorage& storage, int distance, int supportAngle, bool supportEverywhere, int supportDistance);

#endif//RAFT_H
