#pragma once

namespace CepuPhysics
{
  struct RigidPose
  {
    RigidPose() = default;
    RigidPose(const glm::vec3& position, const glm::quat& orientation) : m_Orientation(orientation), m_Position(position) {};
    RigidPose(const glm::vec3& position) : m_Position(position) {};

    static void Transform         (const glm::vec3& v, const RigidPose& pose, glm::vec3& o_result);
    static void TransformByInverse(const glm::vec3& v, const RigidPose& pose, glm::vec3& o_result);
    static void Invert(const RigidPose& pose, RigidPose& o_inverse);
    static void MultiplyWithoutOverlap(const RigidPose& a, const RigidPose& b, RigidPose& o_result);

    glm::quat m_Orientation = glm::quat_identity<float, glm::defaultp>();
    glm::vec3 m_Position    = glm::vec3(0);
  };

  struct BodyInertia
  {
    //@TODO (alektron) Bepu uses an explicit symmetric matrix here to represent tensors
    //Since it's the more compact representation we probably want to do that too in the future.
    //For now this gets us going more quickly
    glm::mat3 m_InverseInertiaTensor = glm::mat3(0);
    float m_InverseMass = 0;

    static BodyInertia Default() { return { glm::mat3(1), 0 }; }
  };

  struct BodyVelocity
  {
    glm::vec3 m_Linear  = glm::vec3(0);
    glm::vec3 m_Angular = glm::vec3(0);
  };

  struct MotionState
  {
    RigidPose m_Pose;
    BodyVelocity m_Velocity;
  };

  struct BodyInertias
  {
    BodyInertia m_Local;
    BodyInertia m_World;
  };

  struct SolverState
  {
    MotionState m_Motion;
    BodyInertias m_Inertia;
  };

  struct RigidPoseWide
  {
    //@TODO (alektron)
  };

  struct BodyActivity
  {
    float m_SleepThreshold = 0;
    uint8_t m_MinimumTimestepsUnderThreshold = 0;

    //Note that all values beyond this point are runtime set. The user should virtually never need to modify them. 
    //We do not constrain write access by default, instead opting to leave it open for advanced users to mess around with.
    //TODO: If people misuse these, we should internalize them in a case by case basis.

    uint8_t m_TimestepsUnderThresholdCount = 0;
    bool m_SleepCandidate = false;
  };
}


