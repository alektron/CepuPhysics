#pragma once
#include "Collidables/CollidableReference.h"
#include "Trees/Tree.h"

namespace CepuPhysics
{
  class BroadPhase
  {
  public:
    BroadPhase(CepuUtil::BufferPool& pool, int32_t initialActiveLeafCapacity = 4096, int32_t initialStaticLeafCapacity = 8192);
    ~BroadPhase();

    static bool RemoveAt(int32_t index, Tree& tree, CepuUtil::Buffer<CollidableReference> leaves, CollidableReference& o_movedLeaf);

    int32_t AddActive(CollidableReference collidable, const CepuUtil::BoundingBox& bounds) { return Add(collidable, bounds, m_ActiveTree, *m_Pool, m_ActiveLeaves); }
    int32_t AddStatic(CollidableReference collidable, const CepuUtil::BoundingBox& bounds) { return Add(collidable, bounds, m_StaticTree, *m_Pool, m_StaticLeaves); }
    bool RemoveActiveAt(int32_t index, CollidableReference& o_movedLeaf) { return RemoveAt(index, m_ActiveTree, m_ActiveLeaves, o_movedLeaf); }
    bool RemoveStaticAt(int32_t index, CollidableReference& o_movedLeaf) { return RemoveAt(index, m_StaticTree, m_StaticLeaves, o_movedLeaf); }

    static void GetBoundsPointers(int32_t broadPhaseIndex, const Tree& tree, glm::vec3** o_minPointer, glm::vec3** o_maxPointer);
    void GetActiveBoundsPointers(int32_t index, glm::vec3** o_minPointer, glm::vec3** o_maxPointer) { return GetBoundsPointers(index, m_ActiveTree, o_minPointer, o_maxPointer); }
    void GetStaticBoundsPointers(int32_t index, glm::vec3** o_minPointer, glm::vec3** o_maxPointer) { return GetBoundsPointers(index, m_StaticTree, o_minPointer, o_maxPointer); }
    
    static void UpdateBounds(int32_t broadPhaseIndex, Tree& tree, const glm::vec3& min, const glm::vec3& max);
    void UpdateActiveBounds(int32_t broadPhaseIndex, const glm::vec3& min, const glm::vec3& max) { return UpdateBounds(broadPhaseIndex, m_ActiveTree, min, max); }
    void UpdateStaticBounds(int32_t broadPhaseIndex, const glm::vec3& min, const glm::vec3& max) { return UpdateBounds(broadPhaseIndex, m_StaticTree, min, max); }
    
    void Update(/*@ IThreadDispatcher threadDispatcher = null*/);
    void Clear();
    
    void EnsureCapacity(int32_t activeCapacity, int32_t staticCapacity);
    
    void Resize(int32_t activeCapacity, int32_t staticCapacity);

  private:
    static int32_t Add(CollidableReference collidable, const CepuUtil::BoundingBox& bounds, Tree& tree, CepuUtil::BufferPool& pol, CepuUtil::Buffer<CollidableReference>& leaves);
    void EnsureCapacity(Tree& tree, CepuUtil::Buffer<CollidableReference>& leaves, int32_t capacity);
    void ResizeCapacity(Tree& tree, CepuUtil::Buffer<CollidableReference>& leaves, int32_t capacity);
    void Dispose       (Tree& tree, CepuUtil::Buffer<CollidableReference>& leaves);

  public:
    CepuUtil::Buffer<CollidableReference> m_ActiveLeaves;
    CepuUtil::Buffer<CollidableReference> m_StaticLeaves;
    CepuUtil::BufferPool* m_Pool = nullptr;

    Tree m_ActiveTree;
    Tree m_StaticTree;

    int32_t m_FrameIndex = 0;
  };
}
