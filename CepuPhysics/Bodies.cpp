#include "CepuPhysicsPCH.h"
#include "Bodies.h"
#include "Collidables/Shapes.h"
#include "CollisionDetection/BroadPhase.h"
#include "BodySet.h"
#include "BodyDescription.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  Bodies::Bodies(CepuUtil::BufferPool* pool, Shapes* shapes, BroadPhase* broadPhase, int32_t initialBodyCapacity, int32_t initialIslandCapacity)
    : m_Pool(pool), m_HandlePool(initialBodyCapacity, pool)
  {
    ResizeHandles(initialBodyCapacity);
    ResizeSetsCapacity(initialIslandCapacity + 1, 0);
    new (GetActiveSet()) BodySet(initialBodyCapacity, pool);
    m_Shapes = shapes;
    m_BroadPhase = broadPhase;
    
    static_assert(CONSTRAINTS_UNSUPPORTED);
  }

  void Bodies::Initialize()
  {
    static_assert(SLEEPING_UNSUPPORTED);
  }

  void Bodies::UpdateBounds(BodyHandle bodyHandle)
  {
    auto& location = m_HandleToLocation[bodyHandle.m_Value];
    auto& set = m_Sets[location.m_SetIndex];
    auto& collidable = set.m_Collidables[location.m_Index];

    if (collidable.m_Shape.Exists()) {
      BoundingBox bodyBounds;
      m_Shapes->ComputeBounds(set.m_SolverStates[location.m_Index].m_Motion.m_Pose, collidable.m_Shape, bodyBounds);
      if (location.m_SetIndex == 0)
        m_BroadPhase->UpdateActiveBounds(collidable.m_BroadPhaseIndex, bodyBounds.m_Min, bodyBounds.m_Max);
      else
        m_BroadPhase->UpdateStaticBounds(collidable.m_BroadPhaseIndex, bodyBounds.m_Min, bodyBounds.m_Max);
    }
  }

  void Bodies::AddCollidableToBroadPhase(BodyHandle bodyHandle, const RigidPose& pose, const BodyInertia& localInertia, Collidable& io_collidable)
  {
    assert(io_collidable.m_Shape.Exists());
    //This body has a collidable; stick it in the broadphase.
    //Note that we have to calculate an initial bounding box for the broad phase to be able to insert it efficiently.
    //(In the event of batch adds, you'll want to use batched AABB calculations or just use cached values.)
    //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
    BoundingBox bodyBounds;
    m_Shapes->ComputeBounds(pose, io_collidable.m_Shape, bodyBounds);
    //Note that new body collidables are always assumed to be active.
    io_collidable.m_BroadPhaseIndex =
      m_BroadPhase->AddActive(
        CollidableReference(IsKinematic(localInertia) ? CollidableMobility::KINEMATIC : CollidableMobility::DYNAMIC, bodyHandle),
        bodyBounds);
  }

  void Bodies::UpdateCollidableBroadPhaseIndex(BodyHandle handle, int32_t newBroadPhaseIndex)
  {
    auto& movedOriginalLocation = m_HandleToLocation[handle.m_Value];
    m_Sets[movedOriginalLocation.m_SetIndex].m_Collidables[movedOriginalLocation.m_Index].m_BroadPhaseIndex = newBroadPhaseIndex;
  }

  void Bodies::RemoveCollidableFromBroadPhase(const Collidable& collidable)
  {
    auto removedBroadPhaseIndex = collidable.m_BroadPhaseIndex;
    //The below removes a body's collidable from the broad phase and adjusts the broad phase index of any moved leaf.
    CollidableReference movedLeaf;
    if (m_BroadPhase->RemoveActiveAt(removedBroadPhaseIndex, movedLeaf)) {
      //Note that this is always an active body, so we know that whatever takes the body's place in the broad phase is also an active body.
      //All statics and inactive bodies exist in the static tree.
      assert(movedLeaf.GetMobility() != CollidableMobility::STATIC);
      UpdateCollidableBroadPhaseIndex(movedLeaf.GetBodyHandle(), removedBroadPhaseIndex);
    }
  }

  BodyHandle Bodies::Add(const BodyDescription& desc)
  {
    assert(m_HandleToLocation.IsAllocated() && "The backing memory of the bodies set should be initialized before use");
    auto handleIndex = m_HandlePool.Take();
    assert(handleIndex <= m_HandleToLocation.GetLength() && "It should be impossible for a new handle to end up more than one slot beyond the current handle to index array. "
      "This would imply some form of resize or compaction bug.");
    if (handleIndex == m_HandleToLocation.GetLength())
      ResizeHandles(m_HandleToLocation.GetLength() << 1);

    assert(glm::abs(glm::length(desc.m_Pose.m_Orientation) - 1) < 1e-6f && "Orientation should be initialized to a unit length quaternion");

    //All new bodies are active for simplicity. Someday, it may be worth offering an optimized path for inactives, but it adds complexity.
    //(Directly adding inactive bodies can be helpful in some networked open world scenarios.)
    auto handle = BodyHandle(handleIndex);

    static_assert(CONSTRAINTS_UNSUPPORTED);
    auto index = GetActiveSet()->Add(desc, handle, 0, m_Pool);
    m_HandleToLocation[handleIndex] = BodyMemoryLocation{ 0, index};

    if (desc.m_Collidable.m_Shape.Exists())
      AddCollidableToBroadPhase(handle, desc.m_Pose, desc.m_LocalInertia, GetActiveSet()->m_Collidables[index]);
    else
      //Don't want to leak undefined data into the collidable state if there's no shape
      GetActiveSet()->m_Collidables[index].m_BroadPhaseIndex = -1;

    return handle;
  }

  bool Bodies::IsKinematic(const BodyInertia& inertia)
  {
    //@TODO (alektron) AVX support?
    return inertia.m_InverseMass == 0 && HasLockedInertia(inertia.m_InverseInertiaTensor);
  }

  bool Bodies::HasLockedInertia(const glm::mat3& inertia)
  {
    //@TODO (alektron) AVX support?
    return inertia[0][0] == 0 &&
           inertia[1][0] == 0 &&
           inertia[1][1] == 0 &&
           inertia[2][0] == 0 &&
           inertia[2][1] == 0 &&
           inertia[2][2] == 0;
  }

  void Bodies::UpdateForKinematicStateChange(BodyHandle handle, BodyMemoryLocation location, BodySet* set, bool previouslyKinematic, bool currentlyKinematic)
  {
    assert(location.m_SetIndex == 0 && "If we're changing kinematic state, we should have already awoken the body");
    if (previouslyKinematic == currentlyKinematic)
      return;

    auto& collidable = set->m_Collidables[location.m_Index];
    if (collidable.m_Shape.Exists()) {
      //Any collidable referneces need their encoded mobility updated
      auto mobility = currentlyKinematic ? CollidableMobility::KINEMATIC : CollidableMobility::DYNAMIC;
      if (location.m_SetIndex == 0)
        m_BroadPhase->m_ActiveLeaves[collidable.m_BroadPhaseIndex] = CollidableReference(mobility, handle);
      else
        m_BroadPhase->m_StaticLeaves[collidable.m_BroadPhaseIndex] = CollidableReference(mobility, handle);

      static_assert(SOLVER_UNSUPPORTED);
      if (currentlyKinematic);
      else;
    }
  }



  void Bodies::SetLocalInertia(BodyHandle handle, const BodyInertia& localInertia)
  {
    auto& location = m_HandleToLocation[handle.m_Value];
    if (location.m_SetIndex > 0) {
      static_assert(SLEEPING_UNSUPPORTED);
    }

    //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move
    auto& set = m_Sets[location.m_SetIndex];
    auto& inertiaRef = set.m_SolverStates[location.m_Index].m_Inertia;
    auto& localInertiaReference = set.m_SolverStates[location.m_Index].m_Inertia.m_Local;
    auto nowKinematic = IsKinematic(localInertia);
    auto previouslyKinematic = IsKinematic(inertiaRef.m_Local);
    inertiaRef.m_Local = localInertia;
    //The world inertia is updated on demand and is not 'persistent' data.
    //In the event that the body is now kinematic, it won't be updated by pose integration and such, so it should be initialized to zeroes.
    //Since initializing it to zeroes unconditionally avoids dynamics having some undefined data lingering around in the worst case, might as well.
    inertiaRef.m_World = BodyInertia();
    UpdateForKinematicStateChange(handle, location, &set, previouslyKinematic, nowKinematic);

  }

  void Bodies::ResizeHandles(int32_t newCapacity)
  {
    newCapacity = BufferPool::GetCapacityForCount<BodyMemoryLocation>(newCapacity);
    if (newCapacity != m_HandleToLocation.GetLength()) {
      auto oldCapacity = m_HandleToLocation.GetLength();
      m_Pool->ResizeToAtLeast(m_HandleToLocation, newCapacity, glm::min(oldCapacity, newCapacity));
      if (m_HandleToLocation.GetLength() > oldCapacity)
        memset(m_HandleToLocation.m_Memory + oldCapacity, 0xFF, sizeof(BodyMemoryLocation) * (m_HandleToLocation.GetLength() - oldCapacity));
    }
  }

  void Bodies::ResizeSetsCapacity(int32_t setsCapacity, int32_t potentiallyAllocatedCount)
  {
    assert(setsCapacity >= potentiallyAllocatedCount && potentiallyAllocatedCount <= m_Sets.GetLength());
    setsCapacity = BufferPool::GetCapacityForCount<BodySet>(setsCapacity);
    if (m_Sets.GetLength() != setsCapacity) {
      auto oldCapacity = m_Sets.GetLength();
      m_Pool->ResizeToAtLeast(m_Sets, setsCapacity, potentiallyAllocatedCount);
      if (oldCapacity < m_Sets.GetLength())
        m_Sets.Clear(oldCapacity, m_Sets.GetLength() - oldCapacity); //We rely on unused slots being default initialized
    }
  }

  bool Bodies::BodyExists(BodyHandle bodyHandle) const
  {
    //A negative set index marks a body handle as unused.
    return bodyHandle.m_Value >= 0 && bodyHandle.m_Value < m_HandleToLocation.GetLength() && m_HandleToLocation[bodyHandle.m_Value].m_SetIndex >= 0;
  }

  void Bodies::ValidateExistingHandle(BodyHandle handle) const
  {
#ifdef _DEBUG
    assert(handle.m_Value >= 0 && "Handles must be nonnegative.");
    assert(handle.m_Value <= m_HandlePool.GetHighestPossiblyClaimedId() && m_HandlePool.GetHighestPossiblyClaimedId() < m_HandleToLocation.GetLength() &&
      "Existing handles must fit within the body handle->index mapping.");
    auto& location = m_HandleToLocation[handle.m_Value];
    assert(location.m_SetIndex >= 0 && location.m_SetIndex < m_Sets.GetLength() && "Body set index must be nonnegative and within the sets buffer length.");
    auto& set = m_Sets[location.m_SetIndex];
    assert(set.IsAllocated());
    assert(set.m_Count <= set.m_IndexToHandle.GetLength());
    assert(location.m_Index >= 0 && location.m_Index < set.m_Count && "Body index must fall within the existing body set.");
    assert(set.m_IndexToHandle[location.m_Index].m_Value == handle.m_Value && "Handle->index must match index->handle map.");
    assert(BodyExists(handle) && "Body must exist according to the BodyExists test.");
#endif
  }

}

