#include "CepuPhysicsPCH.h"

#include "Tree.h"
#include "Node.h"
#include "Tree_SelfQueries.h"

#include "BoundingBox.h"

namespace CepuPhysics
{
  bool Intersects(const NodeChild& a, const NodeChild& b)
  {
    return CepuUtil::BoundingBox::Intersects(a.Max, a.Max, b.Min, b.Max);
  }

}
