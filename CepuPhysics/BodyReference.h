#pragma once
#include "Handles.h"
#include "BodyMemoryLocation.h"
#include "Collidables/BodyProperties.h"
#include "Collidables/CollidableReference.h"

namespace CepuPhysics
{
  class Bodies;
  struct BodyVelocity;
  struct RigidPose   ;
  struct MotionState ;
  struct SolverState ;
  struct Collidable  ;
  struct BodyInertia ; 

  struct BodyReference
  {
    BodyReference(BodyHandle handle, Bodies* bodies);
    bool Exists() const;
    const BodyMemoryLocation& GetMemoryLocation() const;
    BodyVelocity& GetVelocity();
    RigidPose   & GetPose();
    MotionState & GetMotionState();
    SolverState & GetSolverState();
    Collidable  & GetCollidable();
    BodyInertia & GetLocalInertia();

    bool IsKinematic();
    bool HasLockedInertia();
    CollidableReference GetCollidableReference() { return CollidableReference(IsKinematic() ? CollidableMobility::KINEMATIC : CollidableMobility::DYNAMIC, m_Handle); }

    void MakeKinematic();
    void SetLocalInertia(const BodyInertia& localInertia);

    BodyHandle m_Handle;
    Bodies* m_Bodies = nullptr;
  };
}


