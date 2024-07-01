#include "CepuPhysicsPCH.h"
#include "BroadPhase.h"

namespace CepuPhysics
{
  BroadPhase::BroadPhase(CepuUtil::BufferPool& pool, int32_t initialActiveLeafCapacity, int32_t initialStaticLeafCapacity)
    : m_ActiveTree(pool, initialActiveLeafCapacity),
      m_StaticTree(pool, initialActiveLeafCapacity)
  {
    m_Pool = &pool;
    pool.TakeAtLeast(initialActiveLeafCapacity, m_ActiveLeaves);
    pool.TakeAtLeast(initialStaticLeafCapacity, m_StaticLeaves);

    //activeRefineContext = new Tree.RefitAndRefineMultithreadedContext();
    //staticRefineContext = new Tree.RefitAndRefineMultithreadedContext();
  }

  BroadPhase::~BroadPhase()
  {
    Dispose(m_ActiveTree, m_ActiveLeaves);
    Dispose(m_StaticTree, m_StaticLeaves);
  }

  void BroadPhase::Update()
  {
    if (m_FrameIndex == INT32_MAX)
      m_FrameIndex = 0;
    //@TODO (alektron)
    static_assert(MULTITHREADING_UNSUPPORTED);

    m_ActiveTree.RefitAndRefine(m_Pool, m_FrameIndex);
    m_StaticTree.RefitAndRefine(m_Pool, m_FrameIndex);
    m_FrameIndex++;
  }

  int32_t BroadPhase::Add(CollidableReference collidable, const CepuUtil::BoundingBox& bounds, Tree& tree, CepuUtil::BufferPool& pool, CepuUtil::Buffer<CollidableReference>& leaves)
  {
    auto leafIndex = tree.Add(bounds, pool);
    if (leafIndex >= leaves.GetLength())
    {
      pool.ResizeToAtLeast(leaves, tree.m_LeafCount + 1, leaves.GetLength());
    }
    leaves[leafIndex] = collidable;
    return leafIndex;
  }

  bool BroadPhase::RemoveAt(int32_t index, Tree& tree, CepuUtil::Buffer<CollidableReference> leaves, CollidableReference& o_movedLeaf)
  {
    assert(index >= 0);
    auto movedLeafIndex = tree.RemoveAt(index);
    if (movedLeafIndex >= 0)
    {
      o_movedLeaf = leaves[movedLeafIndex];
      leaves[index] = o_movedLeaf;
      return true;
    }
    o_movedLeaf = CollidableReference();
    return false;
  }

  void BroadPhase::GetBoundsPointers(int32_t broadPhaseIndex, const Tree& tree, glm::vec3** o_minPointer, glm::vec3** o_maxPointer)
  {
    auto leaf = tree.m_Leaves[broadPhaseIndex];
    auto nodeChild = (&tree.m_Nodes.m_Memory[leaf.GetNodeIndex()].A) + leaf.GetChildIndex();
    *o_minPointer = &nodeChild->Min;
    *o_maxPointer = &nodeChild->Max;
  }

  void BroadPhase::UpdateBounds(int32_t broadPhaseIndex, Tree& tree, const glm::vec3& min, const glm::vec3& max)
  {
    glm::vec3* minPtr, *maxPtr;
    GetBoundsPointers(broadPhaseIndex, tree, &minPtr, &maxPtr);
    *minPtr = min;
    *maxPtr = max;
    tree.RefitForNodeBoundsChange(tree.m_Leaves[broadPhaseIndex].GetNodeIndex());
  }

  void BroadPhase::Clear()
  {
    m_ActiveTree.Clear();
    m_StaticTree.Clear();
  }

  void BroadPhase::EnsureCapacity(int32_t activeCapacity, int32_t staticCapacity)
  {
    EnsureCapacity(m_ActiveTree, m_ActiveLeaves, activeCapacity);
    EnsureCapacity(m_StaticTree, m_StaticLeaves, staticCapacity);
  }

  void BroadPhase::Resize(int32_t activeCapacity, int32_t staticCapacity)
  {
    ResizeCapacity(m_ActiveTree, m_ActiveLeaves, activeCapacity);
    ResizeCapacity(m_StaticTree, m_StaticLeaves, staticCapacity);
  }

  void BroadPhase::EnsureCapacity(Tree& tree, CepuUtil::Buffer<CollidableReference>& leaves, int32_t capacity)
  {
    if (tree.m_Leaves.GetLength() < capacity)
      tree.Resize(*m_Pool, capacity);
    if (leaves.GetLength() < capacity)
      m_Pool->ResizeToAtLeast(leaves, capacity, tree.m_LeafCount);
  }

  void BroadPhase::ResizeCapacity(Tree& tree, CepuUtil::Buffer<CollidableReference>& leaves, int32_t capacity)
  {
    capacity = glm::max(capacity, tree.m_LeafCount);
    if (tree.m_Leaves.GetLength() != CepuUtil::BufferPool::GetCapacityForCount<Leaf>(capacity))
      tree.Resize(*m_Pool, capacity);
    if (leaves.GetLength() != CepuUtil::BufferPool::GetCapacityForCount<CollidableReference>(capacity))
      m_Pool->ResizeToAtLeast(leaves, capacity, tree.m_LeafCount);
  }

  void BroadPhase::Dispose(Tree& tree, CepuUtil::Buffer<CollidableReference>& leaves)
  {
    m_Pool->Return(leaves);
    tree.Dispose(*m_Pool);
  }
}

