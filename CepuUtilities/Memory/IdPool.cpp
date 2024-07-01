#include "CepuUtilitiesPCH.h"
#include "IdPool.h"

namespace CepuUtil
{
  IdPool::IdPool(int32_t initialCapacity, BufferPool* pool)
  {
    m_NextIndex = 0;
    m_AvailableIdCount = 0;
    pool->TakeAtLeast(initialCapacity, m_AvailableIds);
  }

  int32_t IdPool::Take()
  {
    assert(m_AvailableIds.IsAllocated());
    if (m_AvailableIdCount > 0)
      return m_AvailableIds[--m_AvailableIdCount];
    return m_NextIndex++;
  }

  void IdPool::Return(int32_t id, BufferPool* pool)
  {
    assert(m_AvailableIds.IsAllocated());
    if (m_AvailableIdCount == m_AvailableIds.GetLength()) {
      auto oldAvailableIds = m_AvailableIds;
      pool->TakeAtLeast(glm::max(m_AvailableIdCount * 2, m_AvailableIds.GetLength()), m_AvailableIds);
      oldAvailableIds.CopyTo(0, m_AvailableIds, 0, m_AvailableIdCount);
      pool->Return(oldAvailableIds);
    }
    ReturnUnsafely(id);
  }

  void IdPool::ReturnUnsafely(int32_t id)
  {
    assert(m_AvailableIds.IsAllocated() && m_AvailableIds.GetLength() > m_AvailableIdCount);
    m_AvailableIds[m_AvailableIdCount++] = id;
  }

  void IdPool::Clear()
  {
    m_NextIndex = 0;
    m_AvailableIdCount = 0;
  }

  void IdPool::EnsureCapacity(int32_t count, BufferPool* pool)
  {
    if (m_AvailableIds.IsAllocated()) {
      //If this was disposed, we must explicitly rehydrate it.
      *this = IdPool(count, pool);
    }
    else {
      if (m_AvailableIds.GetLength() < count)
        InternalResize(count, pool);
    }
  }

  void IdPool::Compact(int32_t minimumCount, BufferPool* pool)
  {
    assert(m_AvailableIds.IsAllocated());
    auto targetLength = BufferPool::GetCapacityForCount<int32_t>(glm::max(minimumCount, m_AvailableIdCount));
    if (m_AvailableIds.GetLength() > targetLength)
      InternalResize(targetLength, pool);
  }

  void IdPool::Resize(int32_t count, BufferPool* pool)
  {
    if (!m_AvailableIds.IsAllocated())
    {
      //If this was disposed, we must explicitly rehydrate it.
      *this = IdPool(count, pool);
    }
    else
    {
      auto targetLength = BufferPool::GetCapacityForCount<int32_t>(glm::max(count, m_AvailableIdCount));
      if (m_AvailableIds.GetLength() != targetLength)
        InternalResize(targetLength, pool);
    }
  }

  void IdPool::Dispose(BufferPool* pool)
  {
    pool->Return(m_AvailableIds);
    //This simplifies reuse and makes it harder to use invalid data
    *this = IdPool();
  }

  void IdPool::InternalResize(int32_t newSize, BufferPool* pool)
  {
    auto oldAvailableIds = m_AvailableIds;
    pool->TakeAtLeast(newSize, m_AvailableIds);
    assert(oldAvailableIds.GetLength() != m_AvailableIds.GetLength() && "Did you really mean to resize this? Nothing changed!");
    oldAvailableIds.CopyTo(0, m_AvailableIds, 0, m_AvailableIdCount);
    pool->Return(oldAvailableIds);
  }

}

