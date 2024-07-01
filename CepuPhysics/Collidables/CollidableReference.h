#pragma once
#include "Handles.h"

namespace CepuPhysics
{
  enum class CollidableMobility
  {
    DYNAMIC = 0,
    KINEMATIC = 1,
    STATIC = 2
  };

  struct CollidableReference
  {
    CollidableReference() = default;
    /*internal*/ CollidableReference(CollidableMobility mobility, int32_t handle);
    CollidableReference(CollidableMobility mobility, BodyHandle handle);
    CollidableReference(StaticHandle handle);

    CollidableMobility GetMobility() const;
    BodyHandle   GetBodyHandle  () const;
    StaticHandle GetStaticHandle() const;
    int32_t      GetRawHandle   () const;

    uint32_t m_Packed = 0;
  };

}
