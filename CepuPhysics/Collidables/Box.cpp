#include "CepuPhysicsPCH.h"
#include "Box.h"

namespace CepuPhysics
{
  void Box::ComputeBounds(const glm::quat& orientation, glm::vec3& o_min, glm::vec3& o_max)
  {
    glm::mat3 basis(orientation);
    auto x = m_HalfWidth  * basis[0];
    auto y = m_HalfHeight * basis[1];
    auto z = m_HalfLength * basis[2];
    o_max = glm::abs(x) + glm::abs(y) + glm::abs(z);
    o_min = -o_max;
  }

  void Box::ComputeAngularExpansionData(float& o_maximumRadius, float& o_maximumAngularExpansion)
  {
    o_maximumRadius = glm::sqrt(m_HalfWidth * m_HalfWidth + m_HalfHeight * m_HalfHeight + m_HalfLength * m_HalfLength);

    //@ (alektron) That seems a bit weird...why create vectors if a scalar would do?
    o_maximumAngularExpansion = o_maximumRadius - glm::min(glm::vec4(m_HalfLength), glm::min(glm::vec4(m_HalfHeight), glm::vec4(m_HalfLength))).x;
  }

  BodyInertia Box::ComputeInertia(float mass)
  {
    return BodyInertia();
  }

  bool Box::RayTest(const RigidPose& pose, const glm::vec3& origin, const glm::vec3& direction, float& o_t, glm::vec3& o_normal)
  {
    throw;
    return false;
  }
}
