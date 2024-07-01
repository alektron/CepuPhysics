#pragma once

namespace CepuPhysics
{
  //@DEVIATION (alektron) We put BodyMemoryLocation into a seperate header so we do not have to include the complete Bodies.h everywhere
  struct BodyMemoryLocation
  {
    int32_t m_SetIndex = 0;
    int32_t m_Index    = 0;
  };
}
