#pragma once
#include "Trees/Tree_SelfQueries.h"

namespace CepuPhysics
{
  class ICollidableOverlapFinder
  {
    virtual void DispatchOverlaps(float dt) = 0;
  };

  //The overlap finder requires type knowledge about the narrow phase that the broad phase lacks. Don't really want to infect the broad phase with a bunch of narrow phase dependent 
  //generic parameters, so instead we just explicitly create a type-aware overlap finder to help the broad phase.
  template<typename TNarrowPhaseCallbacks>
  class CollidableOverlapFinder : public ICollidableOverlapFinder
  {
  public:
    struct SelfOverlapHandler : public IOverlapHandler
    {

    };

    struct IntertreeOverlapHandler : public IOverlapHandler
    {

    };

    virtual void DispatchOverlaps(float dt) override
    {
      static_assert(MULTITHREADING_UNSUPPORTED);

      SelfOverlapHandler selfTestHandler(m_BroadPhase->m_ActiveLeaves);
      GetSelfOverlaps(m_BroadPhase->m_ActiveTree, selfTestHandler);
    }

    BroadPhase* m_BroadPhase = nullptr;

  };
}