#include "CepuPhysicsPCH.h"
#include "BodyReference.h"
#include "Bodies.h"
#include "BodySet.h"

namespace CepuPhysics
{
  BodyReference::BodyReference(BodyHandle handle, Bodies* bodies)
    : m_Bodies(bodies), m_Handle(handle) {}

  bool BodyReference::Exists() const
  {
    return m_Bodies->BodyExists(m_Handle);
  }

  const BodyMemoryLocation& BodyReference::GetMemoryLocation() const
  {
    m_Bodies->ValidateExistingHandle(m_Handle);
    return m_Bodies->m_HandleToLocation[m_Handle.m_Value];
  }

  BodyVelocity& BodyReference::GetVelocity()
  {
    auto& location = GetMemoryLocation();
    return m_Bodies->m_Sets[location.m_SetIndex].m_SolverStates[location.m_Index].m_Motion.m_Velocity;
  }

  RigidPose& BodyReference::GetPose()
  {
    auto& location = GetMemoryLocation();
    return m_Bodies->m_Sets[location.m_SetIndex].m_SolverStates[location.m_Index].m_Motion.m_Pose;
  }

  MotionState& BodyReference::GetMotionState()
  {
    auto& location = GetMemoryLocation();
    return m_Bodies->m_Sets[location.m_SetIndex].m_SolverStates[location.m_Index].m_Motion;
  }

  SolverState& BodyReference::GetSolverState()
  {
    auto& location = GetMemoryLocation();
    return m_Bodies->m_Sets[location.m_SetIndex].m_SolverStates[location.m_Index];
  }

  Collidable& BodyReference::GetCollidable()
  {
    auto& location = GetMemoryLocation();
    return m_Bodies->m_Sets[location.m_SetIndex].m_Collidables[location.m_Index];
  }

  BodyInertia& BodyReference::GetLocalInertia()
  {
    auto& location = GetMemoryLocation();
    return m_Bodies->m_Sets[location.m_SetIndex].m_SolverStates[location.m_Index].m_Inertia.m_Local;
  }

  bool BodyReference::IsKinematic()
  {
    return m_Bodies->IsKinematic(GetLocalInertia());
  }

  bool BodyReference::HasLockedInertia()
  {
    return m_Bodies->HasLockedInertia(GetLocalInertia().m_InverseInertiaTensor);
  }

  void BodyReference::MakeKinematic()
  {
    if (!IsKinematic())
      m_Bodies->SetLocalInertia(m_Handle, BodyInertia());
  }

  void BodyReference::SetLocalInertia(const BodyInertia& localInertia)
  {
    m_Bodies->SetLocalInertia(m_Handle, localInertia);
  }




}
