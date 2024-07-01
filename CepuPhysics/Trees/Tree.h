#pragma once
#include "Node.h"
#include "Memory/Buffer.h"

namespace CepuUtil
{
  class BufferPool;
  struct BoundingBox;
}

namespace CepuPhysics
{
  struct BinnedResources;
  struct SubtreeHeapEntry;

  enum class InsertionChoice
  {
    NEW_INTERNAL,
    TRAVERSE
  };

  class Tree
  {
  public:
    Tree(CepuUtil::BufferPool& pool, int32_t initialLeafCapacity = 4096);
    void Dispose(CepuUtil::BufferPool& pool);

    void InitializeRoot();
    void Resize(CepuUtil::BufferPool& pool, int32_t targetLeafSlotCount);
    void Clear();

    int32_t Add(const CepuUtil::BoundingBox& bounds, CepuUtil::BufferPool& pool);
    int32_t MergeLeafNodes(const CepuUtil::BoundingBox& newLeafBounds, int32_t parentIndex, int32_t indexInParent, const CepuUtil::BoundingBox& merged);
    int32_t InsertLeafIntoEmptySlot(const CepuUtil::BoundingBox& leafBox, int32_t nodeIndex, int32_t childIndex, Node& node);

    void RemoveNodeAt   (int32_t nodeIndex);
    void RefitForRemoval(int32_t nodeIndex);
    int32_t RemoveAt(int32_t leafIndex);

    void RefitForNodeBoundsChange(int32_t nodeIndex);
    float RefitAndMeasure(NodeChild& child);
    float RefitAndMark(int32_t leafCountThreshold, std::vector<int32_t>& refinementCandidates, CepuUtil::BufferPool* pool); //@QUICKLIST (alektron)
    float RefitAndMark(NodeChild& child, int32_t leafCountThreshold, std::vector<int32_t>& refinementCandidates, CepuUtil::BufferPool* pool); //@QUICKLIST (alektron)
    void RefitAndRefine(CepuUtil::BufferPool* pool, int32_t frameIndex, float refineAggressivenessScale = 1, float chacheOptimizeAggressivenessScale = 1);

    void GetRefitAndMarkTuning(int32_t& o_maximumSubtrees, int32_t& o_estimatedRefinementChandidateCount, int32_t& o_refinementLeafCountThreshold) const;
    void GetRefineTuning(int32_t frameIndex, int32_t refinementCndidatesCount, float refineAggressivenessScale, float costChange,
      int32_t& o_targetRefinementCount, int32_t& o_refinementPeriod, int32_t& o_refinementOffset) const;

    void CollectSubtrees(int32_t nodeIndex, int32_t maximumSubtrees, SubtreeHeapEntry* entries, std::vector<int32_t>& subtrees, std::vector<int32_t>& internalNodes, float& o_treeletCost);
    void ValidateStaging(Node* stagingNodes, int32_t stagingNodeIndex,
      std::vector<int32_t>& subtreeNodePointers, std::vector<int32_t>& collectedSubtreeReferences, 
      std::vector<int32_t>& internalReferences, CepuUtil::BufferPool* pool, int32_t& foundSubtrees, int32_t& foundLeafCount);

    static void CreateBinnedResources(CepuUtil::BufferPool* pool, int32_t maximumSubtreeCount, CepuUtil::Buffer<uint8_t>& o_buffer, BinnedResources& o_resources);
    int32_t CreateStagingNodeBinned(BinnedResources& resources, int32_t start, int32_t count, int32_t& io_stagingNodeCount, float& io_childTreeletsCost) const;
      void BinnedRefine(int32_t nodeIndex, std::vector<int32_t>& subtreeReferences, int32_t maximumSubtrees, std::vector<int32_t>& treeletInternalNodes,
      BinnedResources& resources, CepuUtil::BufferPool* pool);
    void ReifyStagingNodes(int treeletRootIndex, Node* stagingNodes,
      std::vector<int32_t>& subtrees, std::vector<int32_t>& treeletInternalNodes, int32_t& io_nextInternalNodeIndexToUse);
    void ReifyChildren(int32_t internalNodeIndex, Node* stagingNodes, std::vector<int32_t>& subtrees, std::vector<int32_t>& treeletInternalNodes,
      int32_t& nextInternalNodeIndexToUse);
    int32_t ReifyStagingNode(int32_t parent, int32_t indexInParent, Node* stagingNodes, int32_t stagingNodeIndex,
      std::vector<int>& subtrees, std::vector<int>& treeletInternalNodes,
      int32_t& io_nextInternalNodeIndexToUse);
    void SplitSubtreesIntoChildrenBinned(BinnedResources& resources, int32_t start, int32_t count,
      int32_t stagingNodeIndex, int32_t& stagingNodesCount, float& o_childrenTreeletsCost) const;
    void FindPartitionBinned(const BinnedResources& resources, int32_t start, int32_t count,
      int32_t& o_splitIndex, CepuUtil::BoundingBox& o_a, CepuUtil::BoundingBox& o_b, int32_t& o_leafCountA, int32_t& o_leafCountB) const;
    int32_t GetCacheOptimizeTuning(int32_t maximumSubtrees, float costChange, float cacheOptimizeAggressivenessScale);

    void IncrementalCacheOptimize(int32_t nodeIndex);
    void SwapNodes(int32_t indexA, int32_t indexB);

    static float ComputeBoundsMetric(const CepuUtil::BoundingBox& bounds);
    static float ComputeBoundsMetric(const glm::vec3& min, const glm::vec3& max);

    static void CreateMerged(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB, CepuUtil::BoundingBox& o_merged);
    static InsertionChoice ComputeBestInsertionChoice(const CepuUtil::BoundingBox& bounds, float newLeafCost, const NodeChild& child, CepuUtil::BoundingBox& o_mergedCandidate, float& o_costChange);

    static int32_t Encode(int32_t index) { return -1 - index; }

    void ValidateBounds(int32_t nodeIndex) const;

    int32_t AllocateNode();
    int32_t AddLeaf(int32_t nodeIndex, int32_t childIndex);

    CepuUtil::Buffer<Node>     m_Nodes;
    CepuUtil::Buffer<MetaNode> m_Metanodes;
    CepuUtil::Buffer<Leaf>     m_Leaves;

    int32_t m_NodeCount = 0;
    int32_t m_LeafCount = 0;
  };

}
