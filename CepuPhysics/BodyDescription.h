#pragma once
#include "Collidables/BodyProperties.h"
#include "Collidables/Collidable.h"
#include "Collidables/CollidableDescription.h"

namespace CepuPhysics
{
  struct BodyActivityDescription
  {
    BodyActivityDescription() = default;
    BodyActivityDescription(float sleepThreshold, uint8_t minimumTimestepCountUnderThreshold = 32)
    {
      m_SleepThreshold = sleepThreshold;
      m_MinimumTimestepCountUnderThreshold = minimumTimestepCountUnderThreshold;
    }

    float m_SleepThreshold = 0;
    uint8_t m_MinimumTimestepCountUnderThreshold = 0;
  };

  struct BodyDescription
  {
    RigidPose    m_Pose;
    BodyVelocity m_Velocity;
    BodyInertia  m_LocalInertia;
    CollidableDescription   m_Collidable;
    BodyActivityDescription m_Activity;
  };

}
