#include "CepuPhysicsPCH.h"
#include "Tree.h"
#include "Tree_BinnedRefine.h"
#include "Tree_RefineCommon.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  constexpr const int MAXIMUM_BIN_COUNT = 64;

  void Tree::ReifyChildren(int32_t internalNodeIndex, Node* stagingNodes, std::vector<int32_t>& subtrees, std::vector<int32_t>& treeletInternalNodes,
    int32_t& nextInternalNodeIndexToUse)
  {
    assert(subtrees.size() > 1);
    auto& internalNode = m_Nodes[internalNodeIndex];
    for (int i = 0; i < 2; ++i)
    {
      auto& child = (&internalNode.A)[i];
      if (child.Index >= 0)
      {
        child.Index = ReifyStagingNode(internalNodeIndex, i, stagingNodes, child.Index,
          subtrees, treeletInternalNodes, nextInternalNodeIndexToUse);
      }
      else
      {
        //It's a subtree. Update its pointers.
        auto subtreeIndex = subtrees[Encode(child.Index)];
        child.Index = subtreeIndex;
        if (subtreeIndex >= 0)
        {
          assert(subtreeIndex >= 0 && subtreeIndex < m_NodeCount);
          //Subtree is an internal node. Update its parent pointers.
          auto& metanode = m_Metanodes[subtreeIndex];
          metanode.IndexInParent = i;
          metanode.Parent = internalNodeIndex;

        }
        else
        {
          //Subtree is o_a leaf node. Update its parent pointers.
          auto leafIndex = Encode(subtreeIndex);
          assert(leafIndex >= 0 && leafIndex < m_LeafCount);
          m_Leaves[leafIndex] = Leaf(internalNodeIndex, i);
        }
      }
    }
  }

  int32_t Tree::ReifyStagingNode(int32_t parent, int32_t indexInParent, Node* stagingNodes, int32_t stagingNodeIndex,
    std::vector<int>& subtrees, std::vector<int>& treeletInternalNodes,
    int32_t& io_nextInternalNodeIndexToUse)
  {
    int32_t internalNodeIndex;
    assert(io_nextInternalNodeIndexToUse < treeletInternalNodes.size() &&
      "Binary trees should never run out of available internal nodes when reifying staging nodes; no nodes are created or destroyed during the process.");

    //There is an internal node that we can use.
    //Note that we remove from the end to guarantee that the treelet root does not change location.
    //The CollectSubtrees function guarantees that the treelet root is enqueued first.
    internalNodeIndex = treeletInternalNodes[io_nextInternalNodeIndexToUse++];

    //To make the staging node real, it requires an accurate parent pointer, index in parent, and child indices.
    //Copy the staging node into the real tree.
    //We take the staging node's child bounds, child indices, leaf counts, and child count.
    //The parent and index in parent are provided by the caller.
    auto& stagingNode = stagingNodes[stagingNodeIndex];
    auto& internalNode = m_Nodes[internalNodeIndex];
    internalNode = stagingNode;
    auto& metanode = m_Metanodes[internalNodeIndex];
    metanode.RefineFlag = 0; //The staging node could have contained arbitrary refine flag data.
    metanode.Parent = parent;
    metanode.IndexInParent = indexInParent;


    ReifyChildren(internalNodeIndex, stagingNodes, subtrees, treeletInternalNodes, io_nextInternalNodeIndexToUse);
    return internalNodeIndex;
  }

  void Tree::ReifyStagingNodes(int treeletRootIndex, Node* stagingNodes,
    std::vector<int32_t>& subtrees, std::vector<int32_t>& treeletInternalNodes, int32_t& io_nextInternalNodeIndexToUse)
  {
    //We take the staging node's child bounds, child indices, leaf counts, and child count.
    //The parent and index in parent of the treelet root CANNOT BE TOUCHED.
    //When running on multiple threads, another thread may modify the Parent and IndexInParent of the treelet root.
    auto& internalNode = m_Nodes[treeletRootIndex];
    internalNode.A = stagingNodes->A;
    internalNode.B = stagingNodes->B;
    ReifyChildren(treeletRootIndex, stagingNodes, subtrees, treeletInternalNodes, io_nextInternalNodeIndexToUse);
  }

  int32_t Tree::CreateStagingNodeBinned(BinnedResources& resources, int32_t start, int32_t count, int32_t& io_stagingNodeCount, float& io_childTreeletsCost) const
  {
    auto stagingNodeIndex = io_stagingNodeCount++;
    auto stagingNode = resources.StagingNodes + stagingNodeIndex;

    if (count <= 2)
    {
      //No need to do any sorting. This node can fit every remaining subtree.
      auto localIndexMap = resources.IndexMap + start;
      auto stagingNodeChildren = &stagingNode->A;
      for (int i = 0; i < count; ++i)
      {
        auto subtreeIndex = localIndexMap[i];
        auto& child  = stagingNodeChildren[i];
        auto& bounds = resources.BoundingBoxes[subtreeIndex];
        child.Min = bounds.m_Min;
        child.Max = bounds.m_Max;
        child.LeafCount = resources.LeafCounts[subtreeIndex];
        child.Index = Tree::Encode(subtreeIndex);
      }
      //Because subtrees do not change in size, they cannot change the cost.
      io_childTreeletsCost = 0;
      return stagingNodeIndex;
    }

    SplitSubtreesIntoChildrenBinned(resources, start, count, stagingNodeIndex, io_stagingNodeCount, io_childTreeletsCost);

    return stagingNodeIndex;
  }

  void Tree::SplitSubtreesIntoChildrenBinned(BinnedResources& resources, int32_t start, int32_t count,
    int32_t stagingNodeIndex, int32_t& stagingNodesCount, float& o_childrenTreeletsCost) const
  {
    assert(count > 2);
    int32_t splitIndex, leafCountA, leafCountB;
    BoundingBox aBounds, bBounds;
    FindPartitionBinned( resources, start, count, splitIndex, aBounds, bBounds, leafCountA, leafCountB);

    //Recursion bottomed out. 
    auto stagingNode = resources.StagingNodes + stagingNodeIndex;

    auto& a = stagingNode->A;
    auto& b = stagingNode->B;
    a.Min = aBounds.m_Min;
    a.Max = aBounds.m_Max;
    b.Min = bBounds.m_Min;
    b.Max = bBounds.m_Max;
    a.LeafCount = leafCountA;
    b.LeafCount = leafCountB;

    int subtreeCountA = splitIndex - start;
    int subtreeCountB = start + count - splitIndex;
    float costA, costB;
    if (subtreeCountA > 1)
    {
      a.Index = CreateStagingNodeBinned( resources, start, subtreeCountA,
         stagingNodesCount, costA);
      costA += ComputeBoundsMetric( aBounds); //An internal node was created; measure its cost.
    }
    else
    {
      assert(subtreeCountA == 1);
      //Only one subtree. Don't create another node.
      a.Index = Encode(resources.IndexMap[start]);
      costA = 0;
    }
    if (subtreeCountB > 1)
    {
      b.Index = CreateStagingNodeBinned( resources, splitIndex, subtreeCountB,
         stagingNodesCount, costB);
      costB += ComputeBoundsMetric( bBounds); //An internal node was created; measure its cost.
    }
    else
    {
      assert(subtreeCountB == 1);
      //Only one subtree. Don't create another node.
      b.Index = Encode(resources.IndexMap[splitIndex]);
      costB = 0;
    }

    o_childrenTreeletsCost = costA + costB;
  }

  void Tree::FindPartitionBinned(const BinnedResources& resources, int32_t start, int32_t count,
    int32_t& o_splitIndex, BoundingBox& o_a, BoundingBox& o_b, int32_t& o_leafCountA, int32_t& o_leafCountB) const
  {
    //Initialize the per-axis candidate maps.
    auto localIndexMap = resources.IndexMap + start;
    BoundingBox centroidBoundingBox;
    centroidBoundingBox.m_Min = resources.Centroids[localIndexMap[0]];
    centroidBoundingBox.m_Max = centroidBoundingBox.m_Min;

    for (int i = 1; i < count; ++i)
    {
      auto centroid = resources.Centroids + localIndexMap[i];
      centroidBoundingBox.m_Min = glm::min(*centroid, centroidBoundingBox.m_Min);
      centroidBoundingBox.m_Max = glm::max(*centroid, centroidBoundingBox.m_Max);
    }

    //Bin along all three axes simultaneously.
    BoundingBox nullBoundingBox { glm::vec3(std::numeric_limits<float>::max()), glm::vec3(std::numeric_limits<float>::lowest())};
    auto span = centroidBoundingBox.m_Max - centroidBoundingBox.m_Min;
    const float epsilon = 1e-12f;
    if (span.x < epsilon && span.y < epsilon && span.z < epsilon)
    {
      //All axes are degenerate.
      //This is the one situation in which we can end up with all objects in the same bin.
      //To stop this, just short circuit.
      o_splitIndex = count / 2;
      o_a = nullBoundingBox;
      o_b = nullBoundingBox;
      o_leafCountA = 0;
      o_leafCountB = 0;
      for (int i = 0; i < o_splitIndex; ++i)
      {
        BoundingBox::CreateMerged(o_a, resources.BoundingBoxes[localIndexMap[i]], o_a);
        o_leafCountA += resources.LeafCounts[localIndexMap[i]];
      }
      for (int i = o_splitIndex; i < count; ++i)
      {
        BoundingBox::CreateMerged(o_b, resources.BoundingBoxes[localIndexMap[i]], o_b);
        o_leafCountB += resources.LeafCounts[localIndexMap[i]];
      }
      o_splitIndex += start;
      return;
    }


    //There is no real value in having tons of bins when there are very few children.
    //At low counts, many of them even end up empty.
    //You can get huge speed boosts by simply dropping the bin count adaptively.
    auto binCount = (int32_t)glm::min((float)MAXIMUM_BIN_COUNT, glm::max(count * .25f, 2.0f));

    //Take into account zero-width cases.
    //This will result in degenerate axes all being dumped into the first bin.
    auto inverseBinSize = glm::vec3(
      span.x > epsilon ? binCount / span.x : 0,
      span.y > epsilon ? binCount / span.y : 0,
      span.z > epsilon ? binCount / span.z : 0);
    //inverseBinSize = new glm::vec3(inverseBinSize.X, inverseBinSize.Y, inverseBinSize.Z);

    //If the span along an axis is too small, just ignore it.
    auto maximumBinIndex = glm::vec3((float)(binCount - 1));

    //Initialize bin information.
    for (int i = 0; i < binCount; ++i)
    {
      resources.BinBoundingBoxesX[i] = nullBoundingBox;
      resources.BinBoundingBoxesY[i] = nullBoundingBox;
      resources.BinBoundingBoxesZ[i] = nullBoundingBox;

      resources.BinSubtreeCountsX[i] = 0;
      resources.BinSubtreeCountsY[i] = 0;
      resources.BinSubtreeCountsZ[i] = 0;

      resources.BinLeafCountsX[i] = 0;
      resources.BinLeafCountsY[i] = 0;
      resources.BinLeafCountsZ[i] = 0;
    }

    //auto startAllocateToBins = Stopwatch.GetTimestamp();
    //Allocate subtrees to bins for all axes simultaneously.
    for (int i = 0; i < count; ++i)
    {
      auto subtreeIndex = localIndexMap[i];
      auto binIndices = glm::min((resources.Centroids[subtreeIndex] - centroidBoundingBox.m_Min) * inverseBinSize, maximumBinIndex);
      auto x = (int)binIndices.x;
      auto y = (int)binIndices.y;
      auto z = (int)binIndices.z;

      resources.SubtreeBinIndicesX[i] = x;
      resources.SubtreeBinIndicesY[i] = y;
      resources.SubtreeBinIndicesZ[i] = z;

      auto leafCount = resources.LeafCounts + subtreeIndex;
      auto subtreeBoundingBox = resources.BoundingBoxes + subtreeIndex;

      resources.BinLeafCountsX[x] += *leafCount;
      resources.BinLeafCountsY[y] += *leafCount;
      resources.BinLeafCountsZ[z] += *leafCount;

      ++resources.BinSubtreeCountsX[x];
      ++resources.BinSubtreeCountsY[y];
      ++resources.BinSubtreeCountsZ[z];

      BoundingBox::CreateMerged(resources.BinBoundingBoxesX[x], *subtreeBoundingBox, resources.BinBoundingBoxesX[x]);
      BoundingBox::CreateMerged(resources.BinBoundingBoxesY[y], *subtreeBoundingBox, resources.BinBoundingBoxesY[y]);
      BoundingBox::CreateMerged(resources.BinBoundingBoxesZ[z], *subtreeBoundingBox, resources.BinBoundingBoxesZ[z]);
    }

    //Determine split axes for all axes simultaneously.
    //Sweep from low to high.
    auto lastIndex = binCount - 1;

    resources.ALeafCountsX[0] = resources.BinLeafCountsX[0];
    resources.ALeafCountsY[0] = resources.BinLeafCountsY[0];
    resources.ALeafCountsZ[0] = resources.BinLeafCountsZ[0];
    resources.AMergedX[0] = resources.BinBoundingBoxesX[0];
    resources.AMergedY[0] = resources.BinBoundingBoxesY[0];
    resources.AMergedZ[0] = resources.BinBoundingBoxesZ[0];
    for (int i = 1; i < lastIndex; ++i)
    {
      auto previousIndex = i - 1;
      resources.ALeafCountsX[i] = resources.BinLeafCountsX[i] + resources.ALeafCountsX[previousIndex];
      resources.ALeafCountsY[i] = resources.BinLeafCountsY[i] + resources.ALeafCountsY[previousIndex];
      resources.ALeafCountsZ[i] = resources.BinLeafCountsZ[i] + resources.ALeafCountsZ[previousIndex];
      BoundingBox::CreateMerged(resources.AMergedX[previousIndex], resources.BinBoundingBoxesX[i], resources.AMergedX[i]);
      BoundingBox::CreateMerged(resources.AMergedY[previousIndex], resources.BinBoundingBoxesY[i], resources.AMergedY[i]);
      BoundingBox::CreateMerged(resources.AMergedZ[previousIndex], resources.BinBoundingBoxesZ[i], resources.AMergedZ[i]);
    }

    //Sweep from high to low.
    BoundingBox bMergedX = nullBoundingBox;
    BoundingBox bMergedY = nullBoundingBox;
    BoundingBox bMergedZ = nullBoundingBox;
    int bLeafCountX = 0;
    int bLeafCountY = 0;
    int bLeafCountZ = 0;

    int bestAxis = 0;
    float cost = std::numeric_limits<float>::max();
    auto binSplitIndex = 0;
    o_a = nullBoundingBox;
    o_b = nullBoundingBox;
    o_leafCountA = 0;
    o_leafCountB = 0;


    for (int i = lastIndex; i >= 1; --i)
    {
      int aIndex = i - 1;
      BoundingBox::CreateMerged(bMergedX, resources.BinBoundingBoxesX[i], bMergedX);
      BoundingBox::CreateMerged(bMergedY, resources.BinBoundingBoxesY[i], bMergedY);
      BoundingBox::CreateMerged(bMergedZ, resources.BinBoundingBoxesZ[i], bMergedZ);
      bLeafCountX += resources.BinLeafCountsX[i];
      bLeafCountY += resources.BinLeafCountsY[i];
      bLeafCountZ += resources.BinLeafCountsZ[i];


      //It's possible for a lot of bins in a row to be unpopulated. In that event, the metric isn't defined; don't bother calculating it.
      float costCandidateX = 0, costCandidateY = 0, costCandidateZ = 0;
      if (bLeafCountX > 0 && resources.ALeafCountsX[aIndex] > 0)
      {
        auto metricAX = ComputeBoundsMetric(resources.AMergedX[aIndex]);
        auto metricBX = ComputeBoundsMetric(bMergedX);
        costCandidateX = resources.ALeafCountsX[aIndex] * metricAX + bLeafCountX * metricBX;
      }
      else
        costCandidateX = std::numeric_limits<float>::max();
      if (bLeafCountY > 0 && resources.ALeafCountsY[aIndex] > 0)
      {
        auto metricAY = ComputeBoundsMetric(resources.AMergedY[aIndex]);
        auto metricBY = ComputeBoundsMetric(bMergedY);
        costCandidateY = resources.ALeafCountsY[aIndex] * metricAY + bLeafCountY * metricBY;
      }
      else
        costCandidateY = std::numeric_limits<float>::max();
      if (bLeafCountZ > 0 && resources.ALeafCountsZ[aIndex] > 0)
      {
        auto metricAZ = ComputeBoundsMetric(resources.AMergedZ[aIndex]);
        auto metricBZ = ComputeBoundsMetric(bMergedZ);
        costCandidateZ = resources.ALeafCountsZ[aIndex] * metricAZ + bLeafCountZ * metricBZ;
      }
      else
        costCandidateZ = std::numeric_limits<float>::max();
      if (costCandidateX < costCandidateY && costCandidateX < costCandidateZ)
      {
        if (costCandidateX < cost)
        {
          bestAxis = 0;
          cost = costCandidateX;
          binSplitIndex = i;
          o_a = resources.AMergedX[aIndex];
          o_b = bMergedX;
          o_leafCountA = resources.ALeafCountsX[aIndex];
          o_leafCountB = bLeafCountX;
        }
      }
      else if (costCandidateY < costCandidateZ)
      {
        if (costCandidateY < cost)
        {
          bestAxis = 1;
          cost = costCandidateY;
          binSplitIndex = i;
          o_a = resources.AMergedY[aIndex];
          o_b = bMergedY;
          o_leafCountA = resources.ALeafCountsY[aIndex];
          o_leafCountB = bLeafCountY;
        }
      }
      else
      {
        if (costCandidateZ < cost)
        {
          bestAxis = 2;
          cost = costCandidateZ;
          binSplitIndex = i;
          o_a = resources.AMergedZ[aIndex];
          o_b = bMergedZ;
          o_leafCountA = resources.ALeafCountsZ[aIndex];
          o_leafCountB = bLeafCountZ;
        }
      }

    }


    int* bestBinSubtreeCounts;
    int* bestSubtreeBinIndices;
    switch (bestAxis)
    {
    case 0:
      bestBinSubtreeCounts = resources.BinSubtreeCountsX;
      bestSubtreeBinIndices = resources.SubtreeBinIndicesX;
      break;
    case 1:
      bestBinSubtreeCounts = resources.BinSubtreeCountsY;
      bestSubtreeBinIndices = resources.SubtreeBinIndicesY;
      break;
    default:
      bestBinSubtreeCounts = resources.BinSubtreeCountsZ;
      bestSubtreeBinIndices = resources.SubtreeBinIndicesZ;
      break;
    }
    //Rebuild the index map.

    resources.BinStartIndices[0] = 0;
    resources.BinSubtreeCountsSecondPass[0] = 0;

    for (int i = 1; i < binCount; ++i)
    {
      resources.BinStartIndices[i] = resources.BinStartIndices[i - 1] + bestBinSubtreeCounts[i - 1];
      resources.BinSubtreeCountsSecondPass[i] = 0;
    }

    //auto startIndexMapTime = Stopwatch.GetTimestamp();

    for (int i = 0; i < count; ++i)
    {
      auto index = bestSubtreeBinIndices[i];
      resources.TempIndexMap[resources.BinStartIndices[index] + resources.BinSubtreeCountsSecondPass[index]++] = localIndexMap[i];
    }

    //Update the real index map.
    for (int i = 0; i < count; ++i)
    {
      localIndexMap[i] = resources.TempIndexMap[i];
    }
    //Transform the split index into object indices.
    o_splitIndex = resources.BinStartIndices[binSplitIndex] + start;

  }

  void Tree::BinnedRefine(int32_t nodeIndex, std::vector<int32_t>& subtreeReferences, int32_t maximumSubtrees, std::vector<int32_t>& treeletInternalNodes,
    BinnedResources& resources, CepuUtil::BufferPool* pool)
  {
    float originalTreeletCost = 0;
    assert(subtreeReferences.size() == 0 && "The subtree references list should be empty since it's about to get filled.");
    assert(subtreeReferences.capacity() >= maximumSubtrees && "Subtree references list should have o_a backing array large enough to hold all possible subtrees.");
    assert(treeletInternalNodes.size() == 0 && "The treelet internal nodes list should be empty since it's about to get filled.");
    assert(treeletInternalNodes.capacity() >= maximumSubtrees - 1 && "Internal nodes queue should have a backing array large enough to hold all possible treelet internal nodes.");
    CollectSubtrees(nodeIndex, maximumSubtrees, resources.SubtreeHeapEntries, subtreeReferences, treeletInternalNodes, originalTreeletCost);
    assert(treeletInternalNodes.size() == subtreeReferences.size() - 2 &&
      "Given that this is o_a binary tree, the number of subtree references found must match the internal nodes traversed to reach them. Note that the treelet root is excluded.");
    assert(subtreeReferences.size() <= maximumSubtrees);

    //TODO: There's no reason to use a priority queue based node selection process for MOST treelets. It's only useful for the root node treelet.
    //For the others, we can use a much cheaper collection scheme.
    //CollectSubtreesDirect(nodeIndex, maximumSubtrees,  subtreeReferences,  treeletInternalNodes, out originalTreeletCost);

    //Gather necessary information from nodes.
    for (int i = 0; i < subtreeReferences.size(); ++i)
    {
      resources.IndexMap[i] = i;
      if (subtreeReferences[i] >= 0)
      {
        //It's an internal node.
        auto& subtreeMetanode = m_Metanodes[subtreeReferences[i]];
        auto& parentNode      = m_Nodes[subtreeMetanode.Parent];
        auto& owningChild     = (&parentNode.A)[subtreeMetanode.IndexInParent];
        auto& targetBounds    = resources.BoundingBoxes[i];
        targetBounds.m_Min = owningChild.Min;
        targetBounds.m_Max = owningChild.Max;
        resources.Centroids[i] = owningChild.Min + owningChild.Max;
        resources.LeafCounts[i] = owningChild.LeafCount;
      }
      else
      {
        //It's a leaf node.
        auto& leaf = m_Leaves[Encode(subtreeReferences[i])];
        auto& owningChild  = (&m_Nodes[leaf.GetNodeIndex()].A)[leaf.GetChildIndex()];
        auto& targetBounds = resources.BoundingBoxes[i];
        targetBounds.m_Min = owningChild.Min;
        targetBounds.m_Max = owningChild.Max;
        resources.Centroids[i] = owningChild.Min + owningChild.Max;
        resources.LeafCounts[i] = 1;
      }
    }

    //Now perform a top-down sweep build.
    //TODO: this staging creation section is really the only part that is sweep-specific. The rest is common to any other kind of subtree-collection based refinement. 
    //If you end up making others, keep this in mind.
    int stagingNodeCount = 0;

    float newTreeletCost = 0;
    CreateStagingNodeBinned(resources, 0, (int32_t)subtreeReferences.size(), stagingNodeCount, newTreeletCost);
    //Copy the refine flag over from the treelet root so that it persists.
    resources.RefineFlags[0] = m_Metanodes[nodeIndex].RefineFlag;


    //ValidateStaging(resources.StagingNodes, nodeIndex, subtreeReferences,  sweepSubtrees,  subtreeReferences, parent, indexInParent);

    //Note that updating only when it improves often results in a suboptimal local minimum. Allowing it to worsen locally often leads to better global scores.
    if (true)//newTreeletCost < originalTreeletCost)
    {
      //The refinement is an actual improvement.
      //Apply the staged nodes to real nodes!
      int nextInternalNodeIndexToUse = 0;
      ReifyStagingNodes(nodeIndex, resources.StagingNodes, subtreeReferences, treeletInternalNodes, nextInternalNodeIndexToUse);
    }

    ValidateBounds(nodeIndex);
  }

  uint8_t* Suballocate(uint8_t* Memory, int32_t& memoryAllocated, int32_t byteCount)
  {
    //When creating binned resources, we suballocate from a single buffer. This is partially an artifact of the old implementation, but 
    //it would also be an awful lot of bufferPool.Take invocations. By doing it explicitly and inline, we can reduce a little overhead.
    //(And we only have to return one buffer at the end, rather than... four hundred)
    auto newSize = memoryAllocated + (byteCount + 15) & ~0xF; //Maintaining 16 byte alignment.
    auto toReturn = Memory + memoryAllocated;
    //static bool toggle = false;
    //memset(toReturn, (int)toggle, (byteCount + 15) & ~0xF);
    //toggle = !toggle;
    memoryAllocated = newSize;
    return toReturn;

  }

  void Tree::CreateBinnedResources(BufferPool* pool, int32_t maximumSubtreeCount, Buffer<uint8_t>& o_buffer, BinnedResources& o_resources)
  {
    //TODO: This is a holdover from the pre-BufferPool tree design. It's pretty ugly. While some preallocation is useful (there's no reason to suffer the overhead of 
    //pulling things out of the BufferPool over and over and over again), the degree to which this preallocates has a negative impact on L1 cache for subtree refines.
    int nodeCount = maximumSubtreeCount - 1;
    int bytesRequired =
      16 * (3 + 3 + 1) + sizeof(BoundingBox) * (maximumSubtreeCount + 3 * nodeCount + 3 * MAXIMUM_BIN_COUNT) +
      16 * (6 + 3 + 8) + sizeof(int) * (maximumSubtreeCount * 6 + nodeCount * 3 + MAXIMUM_BIN_COUNT * 8) +
      16 * (1) + sizeof(glm::vec3) * maximumSubtreeCount +
      16 * (1) + sizeof(SubtreeHeapEntry) * maximumSubtreeCount +
      16 * (1) + sizeof(Node) * nodeCount +
      16 * (1) + sizeof(int) * nodeCount;

    pool->TakeAtLeast(bytesRequired, o_buffer);
    memset(o_buffer.m_Memory, 0, bytesRequired);
    auto memory = o_buffer.m_Memory;
    int memoryAllocated = 0;

    o_resources.BoundingBoxes      = (BoundingBox*     )Suballocate(memory, memoryAllocated, sizeof(BoundingBox)      * maximumSubtreeCount);
    o_resources.LeafCounts         = (int*             )Suballocate(memory, memoryAllocated, sizeof(int)              * maximumSubtreeCount);
    o_resources.IndexMap           = (int*             )Suballocate(memory, memoryAllocated, sizeof(int)              * maximumSubtreeCount);
    o_resources.Centroids          = (glm::vec3*       )Suballocate(memory, memoryAllocated, sizeof(glm::vec3)          * maximumSubtreeCount);
    o_resources.SubtreeHeapEntries = (SubtreeHeapEntry*)Suballocate(memory, memoryAllocated, sizeof(SubtreeHeapEntry) * maximumSubtreeCount);
    o_resources.StagingNodes       = (Node*            )Suballocate(memory, memoryAllocated, sizeof(Node)             * nodeCount);
    o_resources.RefineFlags        = (int*             )Suballocate(memory, memoryAllocated, sizeof(int)              * nodeCount);

    o_resources.SubtreeBinIndicesX = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * maximumSubtreeCount);
    o_resources.SubtreeBinIndicesY = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * maximumSubtreeCount);
    o_resources.SubtreeBinIndicesZ = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * maximumSubtreeCount);
    o_resources.TempIndexMap       = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * maximumSubtreeCount);

    o_resources.ALeafCountsX =     (int*)Suballocate(memory, memoryAllocated, sizeof(int)         * nodeCount);
    o_resources.ALeafCountsY =     (int*)Suballocate(memory, memoryAllocated, sizeof(int)         * nodeCount);
    o_resources.ALeafCountsZ =     (int*)Suballocate(memory, memoryAllocated, sizeof(int)         * nodeCount);
    o_resources.AMergedX = (BoundingBox*)Suballocate(memory, memoryAllocated, sizeof(BoundingBox) * nodeCount);
    o_resources.AMergedY = (BoundingBox*)Suballocate(memory, memoryAllocated, sizeof(BoundingBox) * nodeCount);
    o_resources.AMergedZ = (BoundingBox*)Suballocate(memory, memoryAllocated, sizeof(BoundingBox) * nodeCount);

    o_resources.BinBoundingBoxesX = (BoundingBox*)Suballocate(memory, memoryAllocated, sizeof(BoundingBox) * MAXIMUM_BIN_COUNT);
    o_resources.BinBoundingBoxesY = (BoundingBox*)Suballocate(memory, memoryAllocated, sizeof(BoundingBox) * MAXIMUM_BIN_COUNT);
    o_resources.BinBoundingBoxesZ = (BoundingBox*)Suballocate(memory, memoryAllocated, sizeof(BoundingBox) * MAXIMUM_BIN_COUNT);
    o_resources.BinLeafCountsX = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);
    o_resources.BinLeafCountsY = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);
    o_resources.BinLeafCountsZ = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);
    o_resources.BinSubtreeCountsX = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);
    o_resources.BinSubtreeCountsY = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);
    o_resources.BinSubtreeCountsZ = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);
    o_resources.BinStartIndices = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);
    o_resources.BinSubtreeCountsSecondPass = (int*)Suballocate(memory, memoryAllocated, sizeof(int) * MAXIMUM_BIN_COUNT);

    assert(memoryAllocated <= o_buffer.GetLength() && "The allocated buffer should be large enough for all the suballocations.");
  }

}