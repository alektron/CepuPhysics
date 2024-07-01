#pragma once
#include "TypedIndex.h"
#include "Collidable.h"

namespace CepuPhysics
{
  struct CollidableDescription
  {
    TypedIndex m_Shape;
    ContinuousDetection m_Continuity;
  };

}
