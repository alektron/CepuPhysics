#pragma once
#include "Memory/IdPool.h"
#include "Handles.h"
#include "BodyMemoryLocation.h"
#include "BodyReference.h"

namespace CepuPhysics
{
  class Shapes;
  class BroadPhase;
  class BodySet;
  struct Collidable;
  struct BodyDescription;
  struct RigidPose;
  struct BodyInertia;

  class Bodies
  {
  public:
    Bodies(CepuUtil::BufferPool* pool, Shapes* shapes, BroadPhase* broadPhase, int32_t initialBodyCapacity, int32_t initialIslandCapacity);
    //void Initialize(Solver* solver, IslandAwakener* awakener, IslandSleeper* sleeper); //@TODO @SLEEP (alektron)
    void Initialize();

    void UpdateBounds(BodyHandle bodyHandle);
    void AddCollidableToBroadPhase(BodyHandle bodyHandle, const RigidPose& pose, const BodyInertia& localInertia, Collidable& io_collidable);
    void UpdateCollidableBroadPhaseIndex(BodyHandle handle, int32_t newBroadPhaseIndex);
    void RemoveCollidableFromBroadPhase(const Collidable& collidable);
    BodyHandle Add(const BodyDescription& desc);
    void RemoveAt(int32_t activeBodyIndex);
    void Remove(BodyHandle handle);
    
    static bool IsKinematic(const BodyInertia& inertia);
    static bool HasLockedInertia(const glm::mat3& inertia); //@TODO (alektron) Symmetric matrix

    void UpdateForKinematicStateChange(BodyHandle handle, BodyMemoryLocation location, BodySet* set, bool previouslyKinematic, bool currentlyKinematic);
    void UpdateForShapeChange         (BodyHandle handle, int32_t activeBodyIndex, TypedIndex oldShape, TypedIndex newShape);
    void SetShape                     (BodyHandle handle, TypedIndex);
    void ApplyDescription             (BodyHandle handle, const BodyDescription& description);
    void GetDescription               (BodyHandle handle, BodyDescription& o_description) const;
    BodyDescription GetDescription    (BodyHandle handle) const;
    void SetLocalInertia              (BodyHandle handle, const BodyInertia& localInertia);

    void ResizeHandles(int32_t newCapacity);
    void ResizeSetsCapacity(int32_t setsCapacity, int32_t potentiallyAllocatedCount);

    BodySet* GetActiveSet() const { return &m_Sets[0]; }

    BodyReference GetBodyRef(BodyHandle handle) { ValidateExistingHandle(handle); return BodyReference(handle, this); }
    bool BodyExists(BodyHandle bodyHandle) const;
    void ValidateExistingHandle(BodyHandle handle) const;

    CepuUtil::Buffer<BodyMemoryLocation> m_HandleToLocation;
    CepuUtil::IdPool m_HandlePool;
    CepuUtil::Buffer<BodySet> m_Sets;

    Shapes* m_Shapes = nullptr;
    BroadPhase* m_BroadPhase = nullptr;


    CepuUtil::BufferPool* m_Pool = nullptr;
  };
}
