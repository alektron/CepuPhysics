#include "CepuPhysicsPCH.h"
#include "Shapes.h"
#include "BodyProperties.h"
#include "Box.h"

namespace CepuPhysics
{

  void Shapes::ComputeBounds(const RigidPose& pose, TypedIndex shapeIndex, CepuUtil::BoundingBox& o_bounds) const
  {
    //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
    m_Batches[shapeIndex.GetType()]->ComputeBounds(shapeIndex.GetIndex(), pose, o_bounds.m_Min, o_bounds.m_Max);
  }


  ShapeBatch* Box::CreateShapeBatch(CepuUtil::BufferPool* pool, int32_t initialCapacity)
  {
    return new ConvexShapeBatch<Box>(pool, initialCapacity);
  }
}

