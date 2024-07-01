#include "CepuUtilitiesPCH.h"
#include "MathChecker.h"

namespace CepuUtil
{
  namespace MathChecker
  {
    bool IsInvalid(float f)
    {
      return glm::isnan(f) || glm::isinf(f);
    }

  }
}


