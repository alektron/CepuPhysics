#pragma once

namespace CepuPhysics
{
  struct SubtreeHeapEntry;

  //TODO: This type was built on assumptions which are no longer valid today; could avoid this complexity with virtually zero overhead now.
  //This will likely be revisited during the larger tree refinement revamp.
  struct BinnedResources
  {
    //TODO:
    //Note that these preallocations will be significantly larger than L1 for a 256-leaf treelet.
    //That's not ideal. 
    //It's largely because we handle all three axes simultaneously. While that's useful to some degree for low-lanecount SIMD reasons,
    //we should look into alternative forms later. For example, gather->wide SIMD->analyze on a per axis basis.
    //Also, it's not clear that precalculating centroids is a good idea- memory for ALU is rarely wise; we already load the bounding boxes.
    CepuUtil::BoundingBox* BoundingBoxes;
    int32_t* LeafCounts;
    int32_t* IndexMap;
    glm::vec3* Centroids;

    SubtreeHeapEntry* SubtreeHeapEntries;
    Node* StagingNodes;
    int32_t* RefineFlags;

    //The binning process requires a lot of auxiliary memory.
    //Rather than continually reallocating it with stackalloc
    //and incurring its zeroing cost (at least until that's improved),
    //create the resources at the beginning of the refinement and reuse them.

    //Subtree related reusable resources.
    int32_t* SubtreeBinIndicesX;
    int32_t* SubtreeBinIndicesY;
    int32_t* SubtreeBinIndicesZ;
    int32_t* TempIndexMap;

    int32_t* ALeafCountsX;
    int32_t* ALeafCountsY;
    int32_t* ALeafCountsZ;
    CepuUtil::BoundingBox* AMergedX;
    CepuUtil::BoundingBox* AMergedY;
    CepuUtil::BoundingBox* AMergedZ;

    //Bin-space reusable resources.
    CepuUtil::BoundingBox* BinBoundingBoxesX;
    CepuUtil::BoundingBox* BinBoundingBoxesY;
    CepuUtil::BoundingBox* BinBoundingBoxesZ;
    int32_t* BinLeafCountsX;
    int32_t* BinLeafCountsY;
    int32_t* BinLeafCountsZ;
    int32_t* BinSubtreeCountsX;
    int32_t* BinSubtreeCountsY;
    int32_t* BinSubtreeCountsZ;

    int32_t* BinStartIndices;
    int32_t* BinSubtreeCountsSecondPass;
  };
}
