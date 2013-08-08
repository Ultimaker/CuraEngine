#ifndef RAFT_H
#define RAFT_H

void generateRaft(SliceDataStorage& storage, int distance, int supportAngle, bool supportEverywhere, int supportDistance)
{
    ClipperLib::Clipper raftUnion;
    for(unsigned int volumeIdx = 0; volumeIdx < storage.volumes.size(); volumeIdx++)
    {
        if (storage.volumes[volumeIdx].layers.size() < 1) continue;
        SliceLayer* layer = &storage.volumes[volumeIdx].layers[0];
        for(unsigned int i=0; i<layer->parts.size(); i++)
        {
            Polygons raft;
            ClipperLib::OffsetPolygons(layer->parts[i].outline, raft, distance, ClipperLib::jtSquare, 2, false);
            raftUnion.AddPolygon(raft[0], ClipperLib::ptSubject);
        }
    }

    if (supportAngle > -1)
    {
        SupportPolyGenerator supportGenerator(storage.support, 0, supportAngle, supportEverywhere, supportDistance, 0);
        raftUnion.AddPolygons(supportGenerator.polygons, ClipperLib::ptSubject);
    }

    Polygons raftResult;
    raftUnion.Execute(ClipperLib::ctUnion, raftResult, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    for(unsigned int n=0; n<raftResult.size(); n++)
        storage.raftOutline.push_back(raftResult[n]);
}

#endif//RAFT_H
