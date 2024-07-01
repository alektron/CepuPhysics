#include "CepuPhysicsPCH.h"
#include "BodySet.h"
#include "BodyDescription.h"
#include "MathChecker.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  BodySet::BodySet(int32_t initialCapacity, CepuUtil::BufferPool* pool)
  {
    InternalResize(initialCapacity, pool);
  }

  void BodySet::InternalResize(int32_t targetBodyCapacity, CepuUtil::BufferPool* pool)
  {
    assert(targetBodyCapacity > 0 && "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
    //Note that we base the bundle capacities on post-resize capacity of the IndexToHandle array. This simplifies the conditions on allocation, but increases memory use.
    //You may want to change this in the future if memory use is concerning.
    targetBodyCapacity = BufferPool::GetCapacityForCount<int>(targetBodyCapacity);
    assert(m_SolverStates.GetLength() != BufferPool::GetCapacityForCount<RigidPoseWide>(targetBodyCapacity) && "Should not try to use internal resize of the result won't change the size.");
    pool->ResizeToAtLeast(m_SolverStates , targetBodyCapacity, m_Count);
    pool->ResizeToAtLeast(m_IndexToHandle, targetBodyCapacity, m_Count);
    pool->ResizeToAtLeast(m_Collidables  , targetBodyCapacity, m_Count);
    pool->ResizeToAtLeast(m_Activity     , targetBodyCapacity, m_Count);
    //pool->ResizeToAtLeast(m_Constraints  , targetBodyCapacity, Count);
  }

  int32_t BodySet::Add(const BodyDescription& bodyDesc, BodyHandle handle, int32_t minimumConstraintCapacity, CepuUtil::BufferPool* pool)
  {
    auto index = m_Count;
    if (index == m_IndexToHandle.GetLength())
      InternalResize(m_IndexToHandle.GetLength() * 2, pool);

    ++m_Count;
    m_IndexToHandle[index] = handle;
    //Collidable's broad phase index is left unset. The Bodies collection is responsible for attaching that data.
    //@TODO @CONSTRAINTS (alektron) Constraints[index] = new QuickList<BodyConstraintReference>(minimumConstraintCapacity, pool);
    static_assert(CONSTRAINTS_UNSUPPORTED);
    ApplyDescriptionByIndex(index, bodyDesc);

    return index;
  }

  void BodySet::ApplyDescriptionByIndex(int32_t index, const BodyDescription& bodyDesc)
  {
    //@TODO (alektron) replace C# string literal formatting
    assert(!MathChecker::IsInvalid(glm::length2(bodyDesc.m_Pose.m_Position)) && "Invalid body position: {bodyDesc.Pose.Position}");
    assert(glm::abs(1 - glm::length2(bodyDesc.m_Pose.m_Orientation)) < 1e-3f && "Body orientation not unit length: {bodyDesc.Pose.Orientation}");
    assert(!MathChecker::IsInvalid(glm::length2(bodyDesc.m_Velocity.m_Linear)) && "Invalid body linear velocity: {bodyDesc.Velocity.Linear}");
    assert(!MathChecker::IsInvalid(glm::length2(bodyDesc.m_Velocity.m_Angular)) && "Invalid body angular velocity: {bodyDesc.Velocity.Angular}");
    assert(!MathChecker::IsInvalid(
      bodyDesc.m_LocalInertia.m_InverseInertiaTensor[0][0] * bodyDesc.m_LocalInertia.m_InverseInertiaTensor[0][0] +
      bodyDesc.m_LocalInertia.m_InverseInertiaTensor[1][0] * bodyDesc.m_LocalInertia.m_InverseInertiaTensor[1][0] +
      bodyDesc.m_LocalInertia.m_InverseInertiaTensor[1][1] * bodyDesc.m_LocalInertia.m_InverseInertiaTensor[1][1] +
      bodyDesc.m_LocalInertia.m_InverseInertiaTensor[2][0] * bodyDesc.m_LocalInertia.m_InverseInertiaTensor[2][0] +
      bodyDesc.m_LocalInertia.m_InverseInertiaTensor[2][1] * bodyDesc.m_LocalInertia.m_InverseInertiaTensor[2][1] +
      bodyDesc.m_LocalInertia.m_InverseInertiaTensor[2][2] * bodyDesc.m_LocalInertia.m_InverseInertiaTensor[2][2]) && "Invalid body inverse inertia tensor: {bodyDesc.LocalInertia.InverseInertiaTensor}");
    assert(!MathChecker::IsInvalid(bodyDesc.m_LocalInertia.m_InverseMass) && bodyDesc.m_LocalInertia.m_InverseMass >= 0 && "Invalid body inverse mass: {bodyDesc.LocalInertia.InverseMass}");

    auto& state = m_SolverStates[index];
    state.m_Motion.m_Pose     = bodyDesc.m_Pose;
    state.m_Motion.m_Velocity = bodyDesc.m_Velocity;
    state.m_Inertia.m_Local   = bodyDesc.m_LocalInertia;
    //Note that the world inertia is only valid in the velocity integration->pose integration interval, so we don't need to initialize it here for dynamics.
    //Kinematics, though, can have their inertia updates skipped at runtime since the world inverse inertia should always be a bunch of zeroes, so we pre-zero it.
    state.m_Inertia.m_World = BodyInertia::Default();
    auto& collidable = m_Collidables[index];
    collidable.m_Continuity = bodyDesc.m_Collidable.m_Continuity;
    //Note that we change the shape here. If the collidable transitions from shapeless->shapeful or shapeful->shapeless, the broad phase has to be notified 
    //so that it can create/remove an entry. That's why this function isn't public.
    collidable.m_Shape = bodyDesc.m_Collidable.m_Shape;
    //To avoid leaking undefined data, initialize the speculative margin to zero.
    //Under normal circumstances, it should not be possible for any relevant system to see an undefined speculative margin, since PredictBoundingBoxes sets the value.
    //However, corner cases like a body being added asleep and woken by the narrow phase can mean prediction does not run.
    collidable.m_SpeculativeMargin = 0;
    auto& activity = m_Activity[index];
    activity.m_SleepThreshold = bodyDesc.m_Activity.m_SleepThreshold;
    activity.m_MinimumTimestepsUnderThreshold = bodyDesc.m_Activity.m_MinimumTimestepCountUnderThreshold;
    activity.m_TimestepsUnderThresholdCount = 0;
    activity.m_SleepCandidate = false;
  }

}
