#pragma once

namespace CepuUtil
{
  struct BoundingBox
  {
    BoundingBox() = default;
    BoundingBox(const glm::vec3& min, const glm::vec3& max);

    static bool Intersects(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB);
    static bool Intersects(const BoundingBox& a, const BoundingBox& b) { return Intersects(a.m_Min, a.m_Max, b.m_Min, b.m_Max); };
    static void CreateMerged(const glm::vec3& minA, const glm::vec3& maxA, const glm::vec3& minB, const glm::vec3& maxB, glm::vec3& o_min, glm::vec3& o_max);
    static void CreateMerged(const BoundingBox& a, const BoundingBox& b, BoundingBox& o_merged) { CreateMerged(a.m_Min, a.m_Max, b.m_Min, b.m_Max, o_merged.m_Min, o_merged.m_Max);} ;

    glm::vec3 m_Min = glm::vec3(0);
    glm::vec3 m_Max = glm::vec3(0);
  };
}
