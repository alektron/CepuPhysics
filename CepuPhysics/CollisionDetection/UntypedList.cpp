#include "CepuPhysicsPCH.h"
#include "UntypedList.h"

using namespace CepuUtil;

namespace CepuPhysics
{
  UntypedList::UntypedList(int32_t elementSizeInBytes, int32_t initialCapacityInElements, CepuUtil::BufferPool* pool)
    : m_Count(0), m_ByteCount(0), m_ElementSizeInBytes(elementSizeInBytes)
  {
    pool->TakeAtLeast(initialCapacityInElements * elementSizeInBytes, m_Buffer);
  }

  int32_t UntypedList::Allocate(int32_t elementSizeInBytes, int32_t minimumElementCount, CepuUtil::BufferPool* pool)
  {
    auto newSize = m_ByteCount + elementSizeInBytes;

    if (!m_Buffer.IsAllocated()) {
      //This didn't exist at all before; create a new entry for this type.
      m_ElementSizeInBytes = elementSizeInBytes;
      pool->TakeAtLeast(glm::max(newSize, minimumElementCount * elementSizeInBytes), m_Buffer);
    }
    else {
      assert(elementSizeInBytes == m_ElementSizeInBytes);
      if (newSize > m_Buffer.GetLength()) {
        //This will bump up to the next allocated block size, so we don't have to worry about constant micro-resizes.
        Buffer<uint8_t> newBuffer;
        pool->TakeAtLeast(newSize, newBuffer);
        m_Buffer.CopyTo(0, newBuffer, 0, m_Buffer.GetLength());
        pool->Return(m_Buffer);
        //@DEVIATION @PERFORMANCE (alektron)
        //pool->ReturnUnsafely(m_Buffer.Id);
        m_Buffer = newBuffer;
      }
    }
    assert(m_Buffer.GetLength()  >= newSize);
    //If we store only byte count, we'd have to divide to get the element index.
    //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
    //So we just store both. It's pretty cheap and simple.
    m_Count++;
    auto byteIndex = m_ByteCount;
    m_ByteCount = newSize;
    return byteIndex;
  }

}
