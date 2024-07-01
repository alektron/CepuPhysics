#pragma once
#include "IShape.h"

namespace CepuPhysics
{
  struct Box : public IConvexShape
  {
    Box() = default; //@ (alektron) We do not actually want a default constructor but we have to until we solve the "GetTypeId is not static" issue
    Box(float width, float height, float length)
      : m_HalfWidth(width * 0.5f), m_HalfHeight(height * 0.5f), m_HalfLength(height * 0.5f) {}

    virtual int32_t GetTypeId() override { return 2; }; //@ (alektron) Can we make this a static function somehow? It never seems to be called on any object instance
    virtual void ComputeBounds(const glm::quat& orientation, glm::vec3& o_min, glm::vec3& o_max) override;
    virtual void ComputeAngularExpansionData(float& o_maximumRadius, float& o_maximumAngularExpansion) override;
    virtual BodyInertia ComputeInertia(float mass) override;
    virtual bool RayTest(const RigidPose& pose, const glm::vec3& origin, const glm::vec3& direction, float& o_t, glm::vec3& o_normal) override;

    virtual ShapeBatch* CreateShapeBatch(CepuUtil::BufferPool* pool, int32_t initialCapacity) override; //@ (alektron) Can we make this static somehow?

    float GetWidth () { return m_HalfWidth  * 2; }
    float GetHeight() { return m_HalfHeight * 2; }
    float GetLength() { return m_HalfLength * 2; }

    void SetWidth (float v) { m_HalfWidth  = v * 0.5f; }
    void SetHeight(float v) { m_HalfHeight = v * 0.5f; }
    void SetLength(float v) { m_HalfLength = v * 0.5f; }

    float m_HalfWidth  = 0;
    float m_HalfHeight = 0;
    float m_HalfLength = 0;
  };
}
