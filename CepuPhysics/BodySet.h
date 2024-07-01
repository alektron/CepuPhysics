#pragma once
#include "Handles.h"
#include "Collidables/Collidable.h"
#include "Collidables/BodyProperties.h"

namespace CepuPhysics
{
  struct BodyDescription;

  class BodySet {
  public:
    BodySet() = default;
    BodySet(int32_t initialCapacity, CepuUtil::BufferPool* pool);
    void InternalResize(int32_t targetBodyCapacity, CepuUtil::BufferPool* pool);

    int32_t Add(const BodyDescription& bodyDesc, BodyHandle handle, int32_t minimumConstraintCapacity, CepuUtil::BufferPool* pool);

    void ApplyDescriptionByIndex(int32_t index, const BodyDescription& bodyDesc);

    CepuUtil::Buffer<BodyHandle  > m_IndexToHandle;
    CepuUtil::Buffer<SolverState > m_SolverStates;
    CepuUtil::Buffer<Collidable  > m_Collidables;
    CepuUtil::Buffer<BodyActivity> m_Activity;

    int32_t m_Count = 0;
    bool IsAllocated() const { return m_IndexToHandle.IsAllocated(); }
  };
}
