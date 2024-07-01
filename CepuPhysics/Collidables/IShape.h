#pragma once
#include "BodyProperties.h"

namespace CepuPhysics
{
  class ShapeBatch;

  struct IShape
  {
    virtual int32_t GetTypeId() = 0;
    virtual ShapeBatch* CreateShapeBatch(CepuUtil::BufferPool* pool, int32_t initialCapacity) = 0;
  };

  struct IConvexShape : public IShape
  {
    virtual void ComputeBounds(const glm::quat& orientation, glm::vec3& o_min, glm::vec3& o_max) = 0;
    virtual void ComputeAngularExpansionData(float& o_maximumRadius, float& maximumAngularExpansion) = 0;
    virtual BodyInertia ComputeInertia(float mass) = 0;

    virtual bool RayTest(const RigidPose& pose, const glm::vec3& origin, const glm::vec3& direction, float& o_t, glm::vec3& o_normal) = 0;
  };
}
