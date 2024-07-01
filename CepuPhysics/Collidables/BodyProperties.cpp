#include "CepuPhysicsPCH.h"
#include "BodyProperties.h"

namespace CepuPhysics
{
  void RigidPose::Transform(const glm::vec3& v, const RigidPose& pose, glm::vec3& o_result)
  {
    //@DEVIATION @PERFORMANCE
    o_result = glm::rotate(pose.m_Orientation, v) + pose.m_Position;
  }

  void RigidPose::TransformByInverse(const glm::vec3& v, const RigidPose& pose, glm::vec3& o_result)
  {
    auto translated = v - pose.m_Position;
    auto conjugate = glm::conjugate(pose.m_Orientation);
    o_result = glm::rotate(conjugate, translated);
  }

  void RigidPose::Invert(const RigidPose& pose, RigidPose& o_inverse)
  {
    o_inverse.m_Orientation = glm::conjugate(pose.m_Orientation);
    o_inverse.m_Position = glm::rotate(o_inverse.m_Orientation, -pose.m_Position);
  }

  void RigidPose::MultiplyWithoutOverlap(const RigidPose& a, const RigidPose& b, RigidPose& o_result)
  {
    o_result.m_Orientation = a.m_Orientation * b.m_Orientation;
    auto rotatedTranslationA = glm::rotate(b.m_Orientation, a.m_Position);
    o_result.m_Position = rotatedTranslationA + b.m_Position;
  }
}
