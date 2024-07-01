#include "CepuPhysicsPCH.h"
#include "CollidableReference.h"

namespace CepuPhysics
{
  CollidableReference::CollidableReference(CollidableMobility mobility, int32_t handle)
  {
    assert((int)mobility >= 0 && (int)mobility <= 2 && "Hey you, that mobility type doesn't exist. Or we changed something and this needs to be updated.");
    assert(handle >= 0 && handle < 1 << 30 && "Do you actually have more than 2^30 collidables? That seems unlikely.");
    m_Packed = ((uint32_t)mobility << 30) | (uint32_t)handle;
  }

  CollidableReference::CollidableReference(CollidableMobility mobility, BodyHandle handle)
    : CollidableReference(mobility, handle.m_Value)
  {
    assert(mobility == CollidableMobility::DYNAMIC || mobility == CollidableMobility::KINEMATIC && "Creating a collidable reference associated with a body requires a body-related mobility.");
  }

  CollidableReference::CollidableReference(StaticHandle handle)
    :CollidableReference(CollidableMobility::STATIC, handle.m_Value)
  {
  }

  CollidableMobility CollidableReference::GetMobility() const
  {
    return static_cast<CollidableMobility>(m_Packed >> 30);
  }

  BodyHandle CollidableReference::GetBodyHandle() const
  {
    assert(GetMobility() == CollidableMobility::DYNAMIC || GetMobility() == CollidableMobility::KINEMATIC && "Extracting a body handle from a collidable reference requires that the collidable is owned by a body.");
    return BodyHandle(GetRawHandle());
  }

  StaticHandle CollidableReference::GetStaticHandle() const
  {
    assert(GetMobility() == CollidableMobility::STATIC && "Extracting a static handle from a collidable reference requires that the collidable is owned by a static.");
    return StaticHandle(GetRawHandle());
  }

  int32_t CollidableReference::GetRawHandle() const
  {
    return (int)(m_Packed & 0x3FFFFFFF);
  }

}
