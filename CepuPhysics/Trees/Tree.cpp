#include "CepuPhysicsPCH.h"
#include "Tree.h"
#include "Memory/BufferPool.h"
#include "BoundingBox.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  Tree::Tree(BufferPool& pool, int32_t initialLeafCapacity)
  {
    if (initialLeafCapacity <= 0)
      throw "Initial leaf capacity must be positive.";

    Resize(pool, initialLeafCapacity);
  }

  void Tree::Dispose(CepuUtil::BufferPool& pool)
  {
    assert(m_Nodes.IsAllocated() == m_Leaves.IsAllocated() && m_Nodes.IsAllocated() == m_Metanodes.IsAllocated() && "Nodes and leaves should have consistent lifetimes.");
    if (m_Nodes.IsAllocated())
    {
      pool.Return(m_Nodes);
      pool.Return(m_Metanodes);
      pool.Return(m_Leaves);
    }
  }

  void Tree::InitializeRoot()
  {
    //The root always exists, even if there are no children in it. Makes some bookkeeping simpler.
    m_NodeCount = 1;
    auto& rootMetanode = m_Metanodes[0];
    rootMetanode.Parent = -1;
    rootMetanode.IndexInParent = -1;
  }

  void Tree::Resize(CepuUtil::BufferPool& pool, int32_t targetLeafSlotCount)
  {
    //Note that it's not safe to resize below the size of potentially used leaves. If the user wants to go smaller, they'll need to explicitly deal with the leaves somehow first.
    auto leafCapacityForTarget = BufferPool::GetCapacityForCount<Leaf    >(glm::max(m_LeafCount, targetLeafSlotCount));

    auto nodeCapacityForTarget     = BufferPool::GetCapacityForCount<Node    >(glm::max(m_NodeCount, leafCapacityForTarget - 1));
    auto metanodeCapacityForTarget = BufferPool::GetCapacityForCount<MetaNode>(glm::max(m_NodeCount, leafCapacityForTarget - 1));
    bool wasAllocated = m_Leaves.IsAllocated();
    assert(m_Leaves.IsAllocated() == m_Nodes.IsAllocated());
    if (leafCapacityForTarget != m_Leaves.GetLength())
      pool.ResizeToAtLeast(m_Leaves, leafCapacityForTarget, m_LeafCount);

    if (nodeCapacityForTarget != m_Nodes.GetLength())
      pool.ResizeToAtLeast(m_Nodes, nodeCapacityForTarget, m_NodeCount);

    if (metanodeCapacityForTarget != m_Metanodes.GetLength()) {
      pool.ResizeToAtLeast(m_Metanodes, metanodeCapacityForTarget, m_NodeCount);
      m_Metanodes.Clear(m_NodeCount, m_Nodes.GetLength() - m_NodeCount);
    }

    if (!wasAllocated)
      InitializeRoot();
  }

  void Tree::Clear()
  {
    m_LeafCount = 0;
    InitializeRoot();
  }

  int32_t Tree::Add(const CepuUtil::BoundingBox& bounds, BufferPool& pool)
  {
    //The rest of the function assumes we have sufficient room. We don't want to deal with invalidated pointers mid-add.
    if (m_Leaves.GetLength() == m_LeafCount)
    {
      //Note that, while we add 1, the underlying pool will request the next higher power of 2 in bytes that can hold it.
      //Since we're already at capacity, that will be ~double the size.
      Resize(pool, m_LeafCount + 1);
    }

    //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
    int nodeIndex = 0;
    auto newLeafCost = ComputeBoundsMetric(bounds);
    while (true)
    {
      //Which child should the leaf belong to?

      //Give the leaf to whichever node had the least cost change. 
      auto& node = m_Nodes[nodeIndex];
      //This is a binary tree, so the only time a node can have less than full children is when it's the root node.
      //By convention, an empty tree still has a root node with no children, so we do have to handle this case.
      if (m_LeafCount < 2)
      {
        //The best slot will, at best, be tied with inserting it in a leaf node because the change in heuristic cost for filling an empty slot is zero.
        return InsertLeafIntoEmptySlot(bounds, nodeIndex, m_LeafCount, node);
      }
      else
      {
        auto& a = node.A;
        auto& b = node.B;
        BoundingBox mergedA, mergedB;
        float costChangeA, costChangeB;
        auto choiceA = ComputeBestInsertionChoice(bounds, newLeafCost, a, mergedA, costChangeA);
        auto choiceB = ComputeBestInsertionChoice(bounds, newLeafCost, b, mergedB, costChangeB);
        if (costChangeA <= costChangeB)
        {
          if (choiceA == InsertionChoice::NEW_INTERNAL)
          {
            return MergeLeafNodes(bounds, nodeIndex, 0, mergedA);
          }
          else //if (choiceA == BestInsertionChoice.Traverse)
          {
            a.Min = mergedA.m_Min;
            a.Max = mergedA.m_Max;
            nodeIndex = a.Index;
            ++a.LeafCount;
          }
        }
        else
        {
          if (choiceB == InsertionChoice::NEW_INTERNAL)
          {
            return MergeLeafNodes(bounds, nodeIndex, 1, mergedB);
          }
          else //if (choiceB == BestInsertionChoice.Traverse)
          {
            b.Min = mergedB.m_Min;
            b.Max = mergedB.m_Max;
            nodeIndex = b.Index;
            ++b.LeafCount;
          }
        }
      }


    }
  }

  int32_t Tree::MergeLeafNodes(const CepuUtil::BoundingBox& newLeafBounds, int32_t parentIndex, int32_t indexInParent, const CepuUtil::BoundingBox& merged)
  {
    //It's a leaf node.
    //Create a new internal node with the new leaf and the old leaf as children.
    //this is the only place where a new level could potentially be created.

    auto newNodeIndex = AllocateNode();
    auto& newNode     = m_Nodes    [newNodeIndex];
    auto& newMetanode = m_Metanodes[newNodeIndex];
    newMetanode.Parent = parentIndex;
    newMetanode.IndexInParent = indexInParent;
    newMetanode.RefineFlag = 0;
    //The first child of the new node is the old leaf. Insert its bounding box.
    auto& parentNode    = m_Nodes[parentIndex];
    auto& childInParent = *(&parentNode.A + indexInParent);
    newNode.A = childInParent;

    //Insert the new leaf into the second child slot.
    auto& b = newNode.B;
    b.Min = newLeafBounds.m_Min;
    auto leafIndex = AddLeaf(newNodeIndex, 1);
    b.Index = Encode(leafIndex);
    b.Max = newLeafBounds.m_Max;
    b.LeafCount = 1;

    //Update the old leaf node with the new index information.
    auto oldLeafIndex = Encode(newNode.A.Index);
    m_Leaves[oldLeafIndex] = Leaf(newNodeIndex, 0);

    //Update the original node's child pointer and bounding box.
    childInParent.Index = newNodeIndex;
    childInParent.Min = merged.m_Min;
    childInParent.Max = merged.m_Max;
    assert(childInParent.LeafCount == 1);
    childInParent.LeafCount = 2;
    return leafIndex;
  }

  int32_t Tree::InsertLeafIntoEmptySlot(const CepuUtil::BoundingBox& leafBox, int32_t nodeIndex, int32_t childIndex, Node& node)
  {
    auto leafIndex = AddLeaf(nodeIndex, childIndex);
    auto& child = *(&node.A + childIndex);
    child.Min = leafBox.m_Min;
    child.Index = Encode(leafIndex);
    child.Max = leafBox.m_Max;
    child.LeafCount = 1;
    return leafIndex;
  }

  float Tree::ComputeBoundsMetric(const CepuUtil::BoundingBox& bounds)
  {
    return ComputeBoundsMetric(bounds.m_Min, bounds.m_Max);
  }

  float Tree::ComputeBoundsMetric(const glm::vec3& min, const glm::vec3& max)
  {
    //Note that we just use the SAH. While we are primarily interested in volume queries for the purposes of collision detection, the topological difference
    //between a volume heuristic and surface area heuristic isn't huge. There is, however, one big annoying issue that volume heuristics run into:
    //all bounding boxes with one extent equal to zero have zero cost. Surface area approaches avoid this hole simply.
    auto offset = max - min;
    //Note that this is merely proportional to surface area. Being scaled by a constant factor is irrelevant.
    return offset.x * offset.y + offset.y * offset.z + offset.x * offset.z;
  }

  void Tree::CreateMerged(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB, CepuUtil::BoundingBox& o_merged)
  {
    o_merged.m_Min = glm::min(minA, minB);
    o_merged.m_Max = glm::max(maxA, maxB);
  }

  //@TODO Use intrinsic for leading zeroes
  //@TODO Put this somewhere else (original Bepu had a SpanHelper class)
  int32_t GetContainingPowerOf2(int32_t x)
  {
    int n = 32;
    unsigned y;

    y = x >>16; if (y != 0) { n = n -16; x = y; }
    y = x >> 8; if (y != 0) { n = n - 8; x = y; }
    y = x >> 4; if (y != 0) { n = n - 4; x = y; }
    y = x >> 2; if (y != 0) { n = n - 2; x = y; }
    y = x >> 1; if (y != 0) return n - 2;
    return n - x;
  }

  InsertionChoice Tree::ComputeBestInsertionChoice(const CepuUtil::BoundingBox& bounds, float newLeafCost, const NodeChild& child, CepuUtil::BoundingBox& o_mergedCandidate, float& o_costChange)
  {
    CreateMerged(child.Min, child.Max, bounds.m_Min, bounds.m_Max, o_mergedCandidate);
    float newCost = ComputeBoundsMetric(o_mergedCandidate);
    if (child.Index >= 0)
    {
      //Estimate the cost of child node expansions as max(SAH(newLeafBounds), costChange) * log2(child.LeafCount).
      //We're assuming that the remaining tree is balanced and that each level will expand by at least SAH(newLeafBounds). 
      //This might not be anywhere close to correct, but it's not a bad estimate.
      o_costChange = newCost - ComputeBoundsMetric(child.Min, child.Max);
      o_costChange += GetContainingPowerOf2(child.LeafCount) * glm::max(newLeafCost, o_costChange);
      return InsertionChoice::TRAVERSE;
    }
    else
    {
      o_costChange = newCost;
      return InsertionChoice::NEW_INTERNAL;
    }
  }

  void Tree::ValidateBounds(int32_t nodeIndex) const
  {
    auto parentIndex = m_Metanodes[nodeIndex].Parent;
    auto& parent = m_Nodes[parentIndex];
    auto& node = m_Nodes    [nodeIndex];
    auto& meta = m_Metanodes[nodeIndex];
    if (meta.IndexInParent == -1)
      return;

    auto& parentBounds = (&parent.A)[meta.IndexInParent];

    CepuUtil::BoundingBox bbParent(parentBounds.Min, parentBounds.Max);
    CepuUtil::BoundingBox bbThisA  (node.A.Min, node.A.Max);
    CepuUtil::BoundingBox bbThisB  (node.B.Min, node.B.Max);
    CepuUtil::BoundingBox merge;
    CepuUtil::BoundingBox::CreateMerged(bbParent, bbThisA, merge);
    assert(bbParent.m_Min == merge.m_Min && bbParent.m_Max == merge.m_Max);
    CepuUtil::BoundingBox::CreateMerged(bbParent, bbThisB, merge);
    assert(bbParent.m_Min == merge.m_Min && bbParent.m_Max == merge.m_Max);
  }

  int32_t Tree::AllocateNode()
  {
    assert(m_Nodes.GetLength() > m_NodeCount && m_Metanodes.GetLength() > m_NodeCount && 
      "Any attempt to allocate a node should not overrun the allocated nodes. For all operations that allocate nodes, capacity should be preallocated.");
    return m_NodeCount++;
  }

  int32_t Tree::AddLeaf(int32_t nodeIndex, int32_t childIndex)
  {
    assert(m_LeafCount < m_Leaves.GetLength() &&
      "Any attempt to allocate a leaf should not overrun the allocated leaves. For all operations that allocate leaves, capacity should be preallocated.");
    m_Leaves[m_LeafCount] = Leaf(nodeIndex, childIndex);
    return m_LeafCount++;
  }

}

