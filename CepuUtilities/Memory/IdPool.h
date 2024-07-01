#pragma once

namespace CepuUtil
{
  class IdPool
  {
  public:
    IdPool(int32_t initialCapacity, BufferPool* pool);
    int32_t Take();
    void Return(int32_t id, BufferPool* pool);
    void ReturnUnsafely(int32_t id);
    void Clear();
    void EnsureCapacity(int32_t count, BufferPool* pool);
    void Compact(int32_t minimumCount, BufferPool* pool);
    void Resize(int32_t count, BufferPool* pool);
    void Dispose(BufferPool* pool);

    int32_t GetHighestPossiblyClaimedId() const { return m_NextIndex - 1; }
    int32_t GetCapacity() const { return m_AvailableIds.GetLength(); }     
    int32_t IsAllocated() const { return m_AvailableIds.IsAllocated(); }   

  private:
    IdPool() = default;
    void InternalResize(int32_t newSize, BufferPool* pool);

  public:
    int32_t m_AvailableIdCount = 0;
    Buffer<int32_t> m_AvailableIds;

  private:
    int32_t m_NextIndex = 0;
  };
}
