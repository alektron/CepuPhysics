#include "CepuUtilitiesPCH.h"
#include "BoundingBox.h"

namespace CepuUtil
{
  BoundingBox::BoundingBox(const glm::vec3& min, const glm::vec3& max)
    : m_Min(min), m_Max(max)
  {
  }

  bool BoundingBox::Intersects(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB)
  {
    return (maxA.x >= minB.x) & (maxA.y >= minB.y) & (maxA.z >= minB.z) &
           (maxB.x >= minA.x) & (maxB.y >= minA.y) & (maxB.z >= minA.z);
  }

  void BoundingBox::CreateMerged(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB, glm::vec3& o_min, glm::vec3& o_max)
  {
    o_min = glm::min(minA, minB);
    o_max = glm::max(maxA, maxB);
  }
}
